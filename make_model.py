#!/usr/bin/env python3
"""
zdjecia_do_3d.py – fotogrametria: folder ze zdjęciami -> siatka 3D (.ply / .stl / .obj)

Potok przetwarzania:
  1. Przygotowanie zdjęć      (Pillow)    – obrót wg EXIF, zmniejszenie, HEIC -> JPG
  2. Structure from Motion    (pycolmap)  – cechy SIFT, dopasowania, pozycje kamer,
                                            rzadka chmura punktów, bundle adjustment
  3. Usunięcie dystorsji      (pycolmap)  – "prostowanie" zdjęć pod gęstą rekonstrukcję
  4. Multi-View Stereo        (OpenMVS)   – gęsta chmura punktów -> siatka (-> tekstura)
  5. Sprzątanie siatki        (Open3D)    – usunięcie śmieci, zapis do .ply i .stl

Jeśli OpenMVS nie jest zainstalowany, skrypt zrobi siatkę "awaryjną"
z rzadkiej chmury punktów (metoda Poissona w Open3D). To słaba jakość,
służy głównie do sprawdzenia, czy zdjęcia w ogóle się "składają".

Użycie:
  python zdjecia_do_3d.py ŚCIEŻKA_DO_ZDJĘĆ [--wyjscie FOLDER] [--max-rozmiar 2000]
                          [--jakosc 1] [--tekstura] [--wygladzanie]
"""

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

# --------------------------------------------------------------------------
# Importy bibliotek – z czytelnym komunikatem, jeśli czegoś brakuje
# --------------------------------------------------------------------------
try:
    import numpy as np
    import open3d as o3d
    import pycolmap
    from PIL import Image, ImageOps
except ImportError as e:
    sys.exit(f"Brak biblioteki: {e.name}. Zainstaluj: pip install pycolmap open3d pillow pillow-heif")

try:  # obsługa zdjęć HEIC z iPhone'a (opcjonalna)
    from pillow_heif import register_heif_opener
    register_heif_opener()
    HEIC_OK = True
except ImportError:
    HEIC_OK = False

ROZSZERZENIA = {".jpg", ".jpeg", ".png", ".tif", ".tiff", ".heic", ".heif"}

# Miejsca, w których zwykle lądują programy OpenMVS po kompilacji na Macu
OPENMVS_KATALOGI = ["/usr/local/bin/OpenMVS", "/opt/homebrew/bin/OpenMVS",
                    "/usr/local/bin", "/opt/homebrew/bin"]


def log(tekst):
    """Wypisuje komunikat z godziną – żeby było widać postęp długich etapów."""
    print(f"[{time.strftime('%H:%M:%S')}] {tekst}", flush=True)


# --------------------------------------------------------------------------
# ETAP 1: przygotowanie zdjęć
# --------------------------------------------------------------------------
def przygotuj_zdjecia(folder_wej: Path, folder_wyj: Path, max_rozmiar: int) -> int:
    """
    Kopiuje zdjęcia do folderu roboczego jako JPG:
    - obraca je zgodnie z EXIF (telefon zapisuje orientację w metadanych),
    - zmniejsza dłuższy bok do max_rozmiar px (przyspiesza wszystko kilkukrotnie),
    - zachowuje EXIF, z którego COLMAP odczyta ogniskową obiektywu.
    Zwraca liczbę przygotowanych zdjęć.
    """
    folder_wyj.mkdir(parents=True, exist_ok=True)
    pliki = sorted(p for p in folder_wej.iterdir() if p.suffix.lower() in ROZSZERZENIA)
    if any(p.suffix.lower() in {".heic", ".heif"} for p in pliki) and not HEIC_OK:
        sys.exit("Znaleziono pliki HEIC – zainstaluj: pip install pillow-heif")

    for i, p in enumerate(pliki, 1):
        cel = folder_wyj / (p.stem + ".jpg")
        if cel.exists():
            continue  # już przygotowane przy poprzednim uruchomieniu
        img = Image.open(p)
        img = ImageOps.exif_transpose(img)          # fizyczny obrót wg EXIF
        exif = img.getexif()
        exif.pop(0x0112, None)                       # usuń znacznik orientacji (już obrócone)
        img.thumbnail((max_rozmiar, max_rozmiar), Image.LANCZOS)
        img.convert("RGB").save(cel, "JPEG", quality=95, exif=exif.tobytes())
        print(f"\r  zdjęcie {i}/{len(pliki)}", end="", flush=True)
    print()
    return len(pliki)


# --------------------------------------------------------------------------
# ETAP 2: Structure from Motion (pozycje kamer + rzadka chmura punktów)
# --------------------------------------------------------------------------
def structure_from_motion(folder_zdjec: Path, folder_sfm: Path) -> Path:
    """
    Uruchamia SfM w COLMAP:
    - extract_features: wykrywa punkty charakterystyczne SIFT na każdym zdjęciu,
    - match_exhaustive: dopasowuje punkty między każdą parą zdjęć (+ RANSAC),
    - incremental_mapping: wylicza pozycje kamer i punkty 3D, z bundle adjustment.
    CameraMode.SINGLE = wszystkie zdjęcia z jednego aparatu (jeden telefon, zoom 1x).
    Zwraca folder z najlepszą (największą) rekonstrukcją.
    """
    folder_sfm.mkdir(parents=True, exist_ok=True)
    baza = folder_sfm / "database.db"
    if baza.exists():
        baza.unlink()  # zawsze świeża baza, żeby nie mieszać danych z poprzednich prób

    log("SfM 1/3: wykrywanie punktów charakterystycznych (SIFT)...")
    pycolmap.extract_features(baza, folder_zdjec, camera_mode=pycolmap.CameraMode.SINGLE)

    log("SfM 2/3: dopasowywanie punktów między zdjęciami...")
    pycolmap.match_exhaustive(baza)

    log("SfM 3/3: wyznaczanie pozycji kamer i punktów 3D...")
    modele = pycolmap.incremental_mapping(baza, folder_zdjec, folder_sfm)
    if not modele:
        sys.exit("Nie udało się zrekonstruować sceny. Zrób więcej zdjęć z większym "
                 "nakładaniem się kadrów i dobrym oświetleniem.")

    # COLMAP może zbudować kilka osobnych modeli, jeśli części zdjęć się nie łączą.
    # Wybieramy ten, który objął najwięcej zdjęć.
    idx, model = max(modele.items(), key=lambda kv: kv[1].num_reg_images())
    liczba = len(list(folder_zdjec.glob("*.jpg")))
    log(f"Model: {model.num_reg_images()}/{liczba} zdjęć, {model.num_points3D()} punktów 3D")
    if model.num_reg_images() < 0.7 * liczba:
        log("UWAGA: sporo zdjęć nie zostało dopasowanych – wynik może być niepełny.")

    folder_najlepszy = folder_sfm / "najlepszy"
    folder_najlepszy.mkdir(exist_ok=True)
    model.write(folder_najlepszy)
    model.export_PLY(str(folder_sfm / "rzadka_chmura.ply"))  # do podglądu
    return folder_najlepszy


# --------------------------------------------------------------------------
# ETAP 3: usunięcie dystorsji obiektywu
# --------------------------------------------------------------------------
def usun_dystorsje(folder_modelu: Path, folder_zdjec: Path, folder_dense: Path):
    """
    Obiektyw telefonu lekko zakrzywia obraz (dystorsja). SfM ją oszacował,
    a tu "prostujemy" zdjęcia, bo OpenMVS zakłada idealny model kamery.
    Zapisujemy model w formacie tekstowym, który na pewno czyta OpenMVS.
    """
    log("Usuwanie dystorsji obiektywu...")
    if folder_dense.exists():
        shutil.rmtree(folder_dense)
    pycolmap.undistort_images(folder_dense, folder_modelu, folder_zdjec)

    sparse = folder_dense / "sparse"
    model = pycolmap.Reconstruction(sparse)
    for f in sparse.glob("*.bin"):
        f.unlink()
    model.write_text(sparse)


# --------------------------------------------------------------------------
# ETAP 4: gęsta rekonstrukcja i siatka w OpenMVS
# --------------------------------------------------------------------------
def znajdz_openmvs(wlasny_katalog=None):
    """Szuka programów OpenMVS w PATH i typowych katalogach. Zwraca katalog albo None."""
    kandydaci = ([wlasny_katalog] if wlasny_katalog else []) + OPENMVS_KATALOGI
    for k in kandydaci:
        if (Path(k) / "InterfaceCOLMAP").exists():
            return Path(k)
    sciezka = shutil.which("InterfaceCOLMAP")
    return Path(sciezka).parent if sciezka else None


def uruchom(katalog_bin: Path, program: str, argumenty: list, cwd: Path):
    """Uruchamia jeden program OpenMVS i przerywa skrypt, jeśli zakończy się błędem."""
    log(f"OpenMVS: {program} {' '.join(argumenty)}")
    wynik = subprocess.run([str(katalog_bin / program), *argumenty], cwd=cwd)
    if wynik.returncode != 0:
        sys.exit(f"{program} zakończył się błędem (kod {wynik.returncode}).")


def openmvs(katalog_bin: Path, folder_dense: Path, jakosc: int,
            wygladzanie: bool, tekstura: bool) -> Path:
    """
    Kolejne programy OpenMVS:
    - InterfaceCOLMAP:   konwersja wyniku COLMAP do formatu sceny OpenMVS (.mvs),
    - DensifyPointCloud: mapy głębi dla każdego zdjęcia -> gęsta chmura punktów,
    - ReconstructMesh:   chmura punktów -> siatka trójkątów,
    - RefineMesh:        (opcjonalnie) doprecyzowanie siatki na podstawie zdjęć – wolne,
    - TextureMesh:       (opcjonalnie) nałożenie kolorów ze zdjęć -> .obj z teksturą.
    jakosc: 0 = pełna rozdzielczość, 1 = połowa (domyślnie), 2 = ćwierć (najszybciej).
    """
    d = folder_dense
    uruchom(katalog_bin, "InterfaceCOLMAP",
            ["-i", ".", "-o", "scene.mvs", "--image-folder", "images"], d)
    uruchom(katalog_bin, "DensifyPointCloud",
            ["scene.mvs", "-o", "scene_dense.mvs", "--resolution-level", str(jakosc)], d)
    uruchom(katalog_bin, "ReconstructMesh",
            ["scene_dense.mvs", "-o", "scene_mesh.mvs"], d)
    siatka = d / "scene_mesh.ply"

    if wygladzanie:
        uruchom(katalog_bin, "RefineMesh",
                ["scene_dense.mvs", "-m", "scene_mesh.ply", "-o", "scene_refined.mvs",
                 "--resolution-level", str(jakosc + 1)], d)
        siatka = d / "scene_refined.ply"

    if tekstura:
        uruchom(katalog_bin, "TextureMesh",
                ["scene_dense.mvs", "-m", siatka.name, "-o", "scene_textured.mvs",
                 "--export-type", "obj"], d)
    return siatka


# --------------------------------------------------------------------------
# Wariant awaryjny: siatka z rzadkiej chmury (bez OpenMVS)
# --------------------------------------------------------------------------
def siatka_awaryjna(plik_chmury: Path, plik_wyj: Path) -> Path:
    """
    Rekonstrukcja Poissona w Open3D z rzadkiej chmury punktów z SfM.
    Punktów jest mało (tysiące zamiast milionów), więc siatka będzie zgrubna.
    """
    log("Tworzenie siatki awaryjnej (Poisson z rzadkiej chmury)...")
    chmura = o3d.io.read_point_cloud(str(plik_chmury))
    chmura, _ = chmura.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    chmura.estimate_normals()
    chmura.orient_normals_consistent_tangent_plane(15)
    siatka, gestosc = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(chmura, depth=8)
    # Poisson "dopowiada" powierzchnię tam, gdzie nie było punktów – usuwamy te miejsca
    gestosc = np.asarray(gestosc)
    siatka.remove_vertices_by_mask(gestosc < np.quantile(gestosc, 0.1))
    o3d.io.write_triangle_mesh(str(plik_wyj), siatka)
    return plik_wyj


# --------------------------------------------------------------------------
# ETAP 5: sprzątanie i eksport
# --------------------------------------------------------------------------
def posprzataj_i_zapisz(plik_siatki: Path, folder_wyj: Path):
    """
    - usuwa zdublowane i zdegenerowane trójkąty,
    - zostawia tylko największy spójny fragment (odrzuca "latające" kawałki tła),
    - zapisuje model.ply (z kolorami) i model.stl (do druku 3D).
    """
    log("Sprzątanie siatki...")
    s = o3d.io.read_triangle_mesh(str(plik_siatki))
    s.remove_duplicated_vertices()
    s.remove_duplicated_triangles()
    s.remove_degenerate_triangles()
    s.remove_non_manifold_edges()

    klastry, ile_trojkatow, _ = s.cluster_connected_triangles()
    klastry, ile_trojkatow = np.asarray(klastry), np.asarray(ile_trojkatow)
    if len(ile_trojkatow) > 1:
        s.remove_triangles_by_mask(klastry != ile_trojkatow.argmax())
        s.remove_unreferenced_vertices()

    s.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(folder_wyj / "model.ply"), s)
    o3d.io.write_triangle_mesh(str(folder_wyj / "model.stl"), s)
    log(f"Gotowe: {len(s.triangles)} trójkątów -> {folder_wyj/'model.ply'} oraz model.stl")


# --------------------------------------------------------------------------
# Główna funkcja – parsuje argumenty i odpala etapy po kolei
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Zdjęcia -> siatka 3D (COLMAP + OpenMVS)")
    ap.add_argument("zdjecia", type=Path, help="folder ze zdjęciami obiektu")
    ap.add_argument("--wyjscie", type=Path, help="folder wyników (domyślnie: <zdjecia>_3d)")
    ap.add_argument("--max-rozmiar", type=int, default=2000,
                    help="dłuższy bok zdjęć w px (domyślnie 2000; 3000 = dokładniej, wolniej)")
    ap.add_argument("--jakosc", type=int, default=1, choices=[0, 1, 2],
                    help="rozdzielczość gęstej chmury: 0 pełna, 1 połowa (domyślnie), 2 ćwierć")
    ap.add_argument("--wygladzanie", action="store_true", help="RefineMesh – lepsze detale, dużo wolniej")
    ap.add_argument("--tekstura", action="store_true", help="dodatkowo model .obj z kolorową teksturą")
    ap.add_argument("--openmvs", help="katalog z programami OpenMVS (jeśli nie ma ich w PATH)")
    a = ap.parse_args()

    if not a.zdjecia.is_dir():
        sys.exit(f"Folder nie istnieje: {a.zdjecia}")
    wyj = (a.wyjscie or a.zdjecia.with_name(a.zdjecia.name + "_3d")).resolve()
    wyj.mkdir(parents=True, exist_ok=True)
    start = time.time()

    log("Etap 1: przygotowanie zdjęć")
    n = przygotuj_zdjecia(a.zdjecia, wyj / "zdjecia", a.max_rozmiar)
    if n < 3:
        sys.exit("Potrzeba co najmniej kilku zdjęć (zalecane 30–100).")
    log(f"Przygotowano {n} zdjęć")

    log("Etap 2: Structure from Motion")
    model = structure_from_motion(wyj / "zdjecia", wyj / "sfm")

    katalog_mvs = znajdz_openmvs(a.openmvs)
    if katalog_mvs:
        log("Etap 3: usunięcie dystorsji")
        usun_dystorsje(model, wyj / "zdjecia", wyj / "dense")
        log("Etap 4: gęsta rekonstrukcja (OpenMVS) – najdłuższy etap")
        siatka = openmvs(katalog_mvs, wyj / "dense", a.jakosc, a.wygladzanie, a.tekstura)
    else:
        log("UWAGA: nie znaleziono OpenMVS – robię siatkę awaryjną (niska jakość).")
        siatka = siatka_awaryjna(wyj / "sfm" / "rzadka_chmura.ply", wyj / "sfm" / "siatka_awaryjna.ply")

    log("Etap 5: sprzątanie siatki")
    posprzataj_i_zapisz(siatka, wyj)
    if katalog_mvs and a.tekstura:
        log(f"Model z teksturą: {wyj/'dense'/'scene_textured.obj'}")
    log(f"Całość zajęła {(time.time() - start) / 60:.1f} min")


if __name__ == "__main__":
    main()