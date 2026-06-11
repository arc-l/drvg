#!/usr/bin/env bash
# Top-level build script for the DRVG project (Linux/macOS, bash).
#
# Usage:
#   ./build.sh [target]
#
# Targets:
#   paper   Build the LaTeX paper -> latex/main.pdf   (default)
#   code    Build the DRVG code base                  (auto-detects Make/CMake)
#   all     Build the paper and the code
#   clean   Remove build artifacts
set -euo pipefail
cd "$(dirname "$0")"
ROOT="$(pwd)"
TARGET="${1:-paper}"

build_paper() {
    echo "==> Building paper"
    bash "$ROOT/latex/compile.sh"
}

build_code() {
    echo "==> Building code"
    if [[ -f "$ROOT/code/Makefile" ]]; then
        make -C "$ROOT/code"
    elif [[ -f "$ROOT/code/CMakeLists.txt" ]]; then
        cmake -S "$ROOT/code" -B "$ROOT/code/build"
        cmake --build "$ROOT/code/build"
    else
        echo "    (no code build configured yet -- nothing to do)"
    fi
}

clean_all() {
    echo "==> Cleaning"
    bash "$ROOT/latex/compile.sh" --clean
    [[ -d "$ROOT/code/build" ]] && rm -rf "$ROOT/code/build"
    return 0
}

case "$TARGET" in
    paper) build_paper ;;
    code)  build_code ;;
    all)   build_paper; build_code ;;
    clean) clean_all ;;
    *)
        echo "Unknown target: $TARGET" >&2
        echo "Valid targets: paper | code | all | clean" >&2
        exit 2
        ;;
esac
