<#
.SYNOPSIS
    Compile the DRVG LaTeX document with pdflatex (+ bibtex).

.DESCRIPTION
    Runs the standard pdflatex -> bibtex -> pdflatex -> pdflatex sequence so that
    cross-references and the bibliography resolve. The bibtex step is tolerant of
    an empty bibliography (early-stage drafts).

.PARAMETER Main
    Base name of the main .tex file (without extension). Default: main.

.PARAMETER Clean
    Remove LaTeX build artifacts and exit without compiling.

.EXAMPLE
    ./compile.ps1
    ./compile.ps1 -Clean
#>
[CmdletBinding()]
param(
    [string]$Main = "main",
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

$artifactExt = @("aux","bbl","blg","log","out","toc","lof","lot","fls",
                 "fdb_latexmk","synctex.gz","nav","snm","vrb")

# Run from the script's own directory, but restore the caller's location on exit.
Push-Location -Path $PSScriptRoot
try {
    if ($Clean) {
        foreach ($ext in $artifactExt) {
            Get-ChildItem -Filter "*.$ext" -ErrorAction SilentlyContinue | Remove-Item -Force
        }
        Write-Host "Cleaned build artifacts."
        return
    }

    if (-not (Get-Command pdflatex -ErrorAction SilentlyContinue)) {
        Write-Error "pdflatex not found on PATH. Install a TeX distribution (e.g. MiKTeX or TeX Live)."
        exit 1
    }

    function Invoke-PdfLatex {
        pdflatex -interaction=nonstopmode -halt-on-error -file-line-error "$Main.tex"
        if ($LASTEXITCODE -ne 0) {
            Write-Error "pdflatex failed (exit $LASTEXITCODE). See $Main.log for details."
            exit $LASTEXITCODE
        }
    }

    Write-Host "==> pdflatex (pass 1)"
    Invoke-PdfLatex

    if (Get-Command bibtex -ErrorAction SilentlyContinue) {
        Write-Host "==> bibtex"
        bibtex $Main   # tolerate empty/no-citation bibliographies
    }

    Write-Host "==> pdflatex (pass 2)"
    Invoke-PdfLatex
    Write-Host "==> pdflatex (pass 3)"
    Invoke-PdfLatex

    Write-Host "Done. Output: $(Join-Path $PSScriptRoot "$Main.pdf")"
}
finally {
    Pop-Location
}
