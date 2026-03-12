#!/usr/bin/env bash
set -euo pipefail

# Create a reproducible virtual environment at .venv and install dev deps
# Usage: ./scripts/setup_python_env.sh

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VENV_DIR="$ROOT_DIR/.venv"

if [ ! -d "$VENV_DIR" ]; then
  python3 -m venv "$VENV_DIR"
fi

# Activate and upgrade pip
source "$VENV_DIR/bin/activate"
python3 -m pip install --upgrade pip setuptools wheel

if [ -f "$ROOT_DIR/requirements-dev.txt" ]; then
  python3 -m pip install -r "$ROOT_DIR/requirements-dev.txt"
fi

echo "Virtualenv prepared at $VENV_DIR"
echo "Activate with: source $VENV_DIR/bin/activate" 
