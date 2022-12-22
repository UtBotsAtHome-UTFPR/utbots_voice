# utbots_voice: pacote ROS de text-to-speech (TTS) e speech-to-text (STT)
# utbots_voice: ROS package of text-to-speech (TTS) and speech-to-text (STT)
- For TTS, we have implementations of:
    - Coqui TTS (Mozilla TTS successor, bleeding edge TTS)
    - Mimic3 (works well in english and other languages but no portuguese at this moment)
- For STT:
    - We use whisper.cpp (high-performance implementation of OpenAI Whisper)
- Therefore, the nodes in this repository only interface the said programs with ROS

## Setup
- ### Clone package repository
    ```bash
    git clone https://github.com/UtBotsAtHome-UTFPR/utbots_voice.git
    ```
- ### Setup whisper.cpp (https://github.com/ggerganov/whisper.cpp)
    ```bash
    # Clone whisper.cpp
    roscd utbots_voice
    git clone https://github.com/ggerganov/whisper.cpp.git
    cd whisper.cpp/

    # Compile
    make

    # Download models
    roscd utbots_voice
    mkdir -p resources/models/
    # Base model (english only)
    wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin -O resources/models/ggml-base.en.bin
    # Model that works for other languages
    wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.bin -O resources/models/ggml-base.bin

    # Install Python dependencies of audio processing
    python3 -m pip install soundfile librosa noisereduce

    # Run the program
    rosrun utbots_voice ros_stt.py
    ```
- ### Mimic3 (https://github.com/MycroftAI/mimic3)
    ```bash
    # Baixar mimic3
    cd ~/Downloads
    wget https://github.com/MycroftAI/mimic3/releases/download/release%2Fv0.2.4/mycroft-mimic3-tts_0.2.4_amd64.deb
    # Instalar pelo apt
    sudo apt install ./mycroft-mimic3-tts_0.2.4_amd64.deb
    ```

- ### Coqui TTS (https://github.com/coqui-ai/TTS)
    ```bash
    # Instalar o TTS
    python3 -m pip install TTS
    # Baixar modelos (ficarão na pasta ~/.local/share/tts/)
    cd /tmp
    tts --text "This command will download models" --model_name "tts_models/en/ljspeech/tacotron2-DDC" --vocoder_name "vocoder_models/en/ljspeech/hifigan_v2" --out_path speech.wav
    tts  --text "Isso vai baixar os modelos" --model_name tts_models/multilingual/multi-dataset/your_tts  --speaker_wav speech.wav --language_idx "pt-br"
    ```


        