#!/bin/bash

set -e  # Exit on error

# === Step 1: Extract the Python executable path ===
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

# === Step 2: Install RNNoise ===
echo "Cloning and building RNNoise..."
cd ~/
if [ ! -d "rnnoise" ]; then
  git clone https://github.com/xiph/rnnoise.git
fi

cd rnnoise
./autogen.sh
./configure
make

# === Step 3: Install RNNoise_Wrapper ===
echo "Installing RNNoise_Wrapper..."
"$PYTHON_EXEC" -m pip install --upgrade pip
"$PYTHON_EXEC" -m pip install git+https://github.com/Desklop/RNNoise_Wrapper.git

# === Step 4: Install Silero VAD dependencies ===
echo "Installing Silero VAD dependencies..."

# Assuming requirements.txt is in the same directory as the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REQUIREMENTS_FILE="$SCRIPT_DIR/requirements.txt"

if [ ! -f "$REQUIREMENTS_FILE" ]; then
  echo "Error: requirements.txt not found at $REQUIREMENTS_FILE"
  exit 1
fi

"$PYTHON_EXEC" -m pip install -r "$REQUIREMENTS_FILE"

# Install PortAudio system dependency
echo "Installing portaudio19-dev..."
sudo apt update
sudo apt install -y portaudio19-dev

# Install PyAudio
"$PYTHON_EXEC" -m pip install pyaudio

echo "✅ Silero setup complete."
