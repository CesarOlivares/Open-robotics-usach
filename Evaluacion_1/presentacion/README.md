# Presentación — Proyecto Octavius

Presentación del proyecto con la plantilla Beamer **USACH theme** (tema Trigon), sin animaciones ni transiciones.

| Archivo | Contenido |
|---|---|
| `Presentacion_Octavius.pdf` | Presentación compilada (24 láminas de exposición + anexos A–B y referencias) |
| `main.tex` | Preámbulo de la plantilla: datos de portada, colores y paquetes |
| `frames.tex` | Contenido: cada tema es un par *diagrama de la lógica → detalle con el código real* |
| `referencias.bib` | Referencias en formato IEEE |
| `*.jpg`, `*.png`, `trigon_*.pdf` | Recursos de la plantilla USACH |

Las figuras se toman de `../docs/img/`, las mismas de `DOCUMENTACION.md`.

## Compilar

Requiere TeX Live completo (paquetes `beamertheme-trigon`, `sourcesanspro`, `listings`, `tikz`). Sin instalar nada, con Docker:

```bash
cd Evaluacion_1
docker run --rm -u $(id -u):$(id -g) -v $PWD:/w -w /w/presentacion texlive/texlive:latest latexmk -pdf main.tex
cp presentacion/main.pdf presentacion/Presentacion_Octavius.pdf
```

También compila en Overleaf (web) subiendo esta carpeta junto con `docs/img/`.
