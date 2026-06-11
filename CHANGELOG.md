# Changelog

All notable changes to this project are documented here. Each entry summarizes
changes relative to the previous committed version. Versioning starts at `0000`.

## [0000] — 2026-06-11

Initial scaffolding commit. Establishes repository structure, the LaTeX paper
skeleton, and build tooling. No algorithmic results yet.

### Added
- Project scaffolding: `docs/claude_input.md` (chat input log), `README.md`,
  `CHANGELOG.md`.
- `code/` directory (with `.gitkeep`) to host the DRVG implementation.
- `latex/` directory for the paper draft:
  - `main.tex` — minimal compilable skeleton titled "Dynamic Rotation-Stacked
    Visibility Graph (DRVG)", wired to the bibliography.
  - `references.bib` — empty bibliography placeholder.
  - `compile.ps1` / `compile.sh` — pdflatex (+ bibtex) build scripts with a
    `-Clean`/`--clean` mode; verified to produce `main.pdf` via MiKTeX.
  - `.gitignore` — excludes LaTeX build artifacts.
- `build.sh` — top-level Linux/macOS bash build script with `paper` (default),
  `code` (auto-detects Make/CMake), `all`, and `clean` targets; validated under
  WSL bash (LF line endings, syntax check, target dispatch).
