# apollo_voice: pacotes ROS de text-to-speech (TTS) e speech-to-text (STT)
- Para TTS, temos implementações de:
    - Coqui TTS (sucessor do Mozilla TTS)
    - Mimic3 (não tem português mas funciona muito bem em inglês)
- Para STT, usamos whisper.cpp (implementação de alta performance do OpenAI Whisper)
- Portanto, os pacotes desse repositório apenas implementam uma interface via ROS para os programas referidos

## Setup
- ### Clonar repositório pacote
    - ```git clone https://github.com/UtBot-UTFPR/apollo_voice.git```
- ### whisper.cpp (https://github.com/ggerganov/whisper.cpp)
    ```bash
    # Clonar repositório do whisper.cpp
    roscd apollo_voice
    git clone https://github.com/ggerganov/whisper.cpp.git
    cd whisper.cpp/
    # Compilar
    make
    # Baixar modelos
    roscd apollo_voice
    mkdir -p resources/models/
    wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin -O resources/models/ggml-base.en.bin
    wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.bin -O resources/models/ggml-base.bin
    # Instalar dependências de processamento de áudio
    python3 -m pip install soundfile librosa noisereduce
    # Executar o programa
    rosrun apollo_voice ros_stt.py
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


        