#!/usr/bin/env bash
set -euo pipefail
mkdir -p output
latexmk -xelatex -pvc -outdir=output -auxdir=output sustech_beamer.tex
