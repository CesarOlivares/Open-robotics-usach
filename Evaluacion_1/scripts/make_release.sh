#!/usr/bin/env bash
# Genera el archivo de entrega: cesar_olivares_2026_2.zip
#
# El zip contiene una carpeta cesar_olivares_2026_2/ con los archivos de
# esta evaluación que están en git (último commit). Así nunca se incluyen
# build/, install/, log/ ni archivos de prueba sueltos, y el contenido es
# exactamente lo versionado. El repositorio puede tener otras carpetas al
# lado (otras evaluaciones): solo se empaqueta la de este proyecto.
#
# Uso (desde cualquier carpeta):
#   ./scripts/make_release.sh            → crea el zip en la raíz del proyecto
#   ./scripts/make_release.sh /ruta/out  → crea el zip en /ruta/out

set -euo pipefail

NAME="cesar_olivares_2026_2"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${1:-$ROOT}"
ZIP="$OUT_DIR/$NAME.zip"

cd "$ROOT"

# Los cambios sin commit NO entran en el zip: avisar para no entregar una
# versión antigua por error.
if [ -n "$(git status --porcelain -- .)" ]; then
  echo "AVISO: hay cambios sin commit; el zip solo incluye el último commit:"
  git status --short -- .
  echo
fi

mkdir -p "$OUT_DIR"
# Se ejecuta desde la carpeta de la evaluación: en una subcarpeta,
# git archive incluye solo esa subcarpeta, con rutas relativas a ella.
git archive --format=zip --prefix="$NAME/" -o "$ZIP" HEAD

echo "Creado: $ZIP ($(du -h "$ZIP" | cut -f1), commit $(git rev-parse --short HEAD))"
