#!/bin/bash

set -e  # Exit on error

# Extract the executable path from setup.cfg
PYTHON_EXEC=$(awk -F= '/^executable *=/ {gsub(/ /,"",$2); print $2}' setup.cfg)

if [ -z "$PYTHON_EXEC" ]; then
  echo "Error: Could not find 'executable' in [build_scripts] of setup.cfg"
  exit 1
fi

if [ ! -x "$PYTHON_EXEC" ]; then
  echo "Error: Specified Python executable does not exist or is not executable: $PYTHON_EXEC"
  exit 1
fi

echo "Using Python interpreter: $PYTHON_EXEC"

# Upgrade pip and install packages using the specified Python interpreter
"$PYTHON_EXEC" -m pip install --upgrade pip
"$PYTHON_EXEC" -m pip install --upgrade transformers accelerate

# Optional install
"$PYTHON_EXEC" -m pip install flash-attn --no-build-isolation || \
  echo "Warning: flash-attn installation failed (expected on some systems)."

echo "✅ Setup complete."
