# 3d_modelling

Fotogrametria: folder ze zdjęciami obiektu -> siatka 3D (`.ply` / `.stl` / `.obj`).

Potok: Pillow (przygotowanie zdjęć) -> pycolmap (Structure from Motion) ->
OpenMVS (gęsta chmura, siatka, tekstura) -> Open3D (sprzątanie i eksport).
Bez OpenMVS skrypt robi siatkę awaryjną (Poisson z rzadkiej chmury) – niska jakość.

## Instalacja

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
brew install libusb      # macOS: wymagane przez open3d
```

OpenMVS trzeba skompilować ze źródeł (https://github.com/cdcseacave/openMVS)
i podać katalog z programami przez `--openmvs` lub dodać go do PATH.

## Użycie

```bash
python make_model.py FOLDER_ZE_ZDJĘCIAMI [--wyjscie FOLDER] [--max-rozmiar 2000]
                     [--jakosc 1] [--wygladzanie] [--tekstura] [--openmvs KATALOG]
```

Wyniki trafiają do `<folder>_3d/` (`model.ply`, `model.stl`).

## Wskazówki dotyczące zdjęć

- 40–80 zdjęć, 2–3 okrążenia wokół obiektu na różnych wysokościach.
- Nakładanie się kadrów > 70%, krok ok. 10–15° między zdjęciami.
- Matowy, nieruchomy obiekt, równe oświetlenie, tło z fakturą.
- Ostre zdjęcia, stała ogniskowa (bez zoomu).
