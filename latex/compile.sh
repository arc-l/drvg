#!/usr/bin/env bash
# Compile the DRVG LaTeX document with pdflatex (+ bibtex).
# Usage: ./compile.sh [main]      compile <main>.tex (default: main)
#        ./compile.sh --clean     remove build artifacts and exit
set -euo pipefail
cd "$(dirname "$0")"

MAIN="${1:-main}"
ARTIFACT_EXT=(aux bbl blg log out toc lof lot fls fdb_latexmk synctex.gz nav snm vrb)

if [[ "${1:-}" == "--clean" ]]; then
    for ext in "${ARTIFACT_EXT[@]}"; do
        rm -f ./*."$ext"
    done
    echo "Cleaned build artifacts."
    exit 0
fi

if ! command -v pdflatex >/dev/null 2>&1; then
    echo "Error: pdflatex not found on PATH. Install TeX Live or MiKTeX." >&2
    exit 1
fi

run_pdflatex() {
    pdflatex -interaction=nonstopmode -halt-on-error -file-line-error "$MAIN.tex"
}

echo "==> pdflatex (pass 1)"
run_pdflatex
if command -v bibtex >/dev/null 2>&1; then
    echo "==> bibtex"
    bibtex "$MAIN" || true   # tolerate empty/no-citation bibliographies
fi
echo "==> pdflatex (pass 2)"
run_pdflatex
echo "==> pdflatex (pass 3)"
run_pdflatex

echo "Done. Output: $MAIN.pdf"
