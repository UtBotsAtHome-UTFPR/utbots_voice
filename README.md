# utbots_voice: ROS package of text-to-speech (TTS) and speech-to-text (STT)
- For TTS:
    - We have a ROS interface of Coqui TTS (Mozilla TTS successor, bleeding edge TTS).
    - We have a ROS interface of Mimic3 (works well in english and other languages but no portuguese at this moment).
    - You probably don't need two TTS programs. For example, Mimic3 will be very good if only english is needed.
- For STT:
    - We have a ROS interface of whisper.cpp (high-performance implementation of OpenAI Whisper).

## Setup
- ### Clone repository and compile workspace
    ```bash
    # Clone repository
    git clone https://github.com/UtBotsAtHome-UTFPR/utbots_voice.git

    # Compile workspace
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
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
    cd resources/models/
    # Base model (english only)
    wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin -O ./ggml-base.en.bin
    # Model that works for other languages
    wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.bin -O ./ggml-base.bin

    # Install Python dependencies of audio processing
    python3 -m pip install soundfile librosa noisereduce

    # Run the test program (it tries to transcript wav from resources/wav/samples/baka_gaijin.wav)
    rosrun utbots_voice ros_stt.py
    ```
- ### Setup Mimic3 (https://github.com/MycroftAI/mimic3)
    ```bash
    # Download mimic3
    cd ~/Downloads
    wget https://github.com/MycroftAI/mimic3/releases/download/release%2Fv0.2.4/mycroft-mimic3-tts_0.2.4_amd64.deb

    # Install with apt
    sudo apt install ./mycroft-mimic3-tts_0.2.4_amd64.deb

    # Test Mimic3 (it should take a small time to download the model)
    mimic3 --voice en_US/hifi-tts_low 'Hello world' | aplay

    # Test the Mimic3 ROS node
    rosrun utbots_voice ros_tts_mimic.py
    rostopic pub /voice/tts/text std_msgs/String 'Hello world' # (in another terminal)
    ```

- ### Coqui TTS (https://github.com/coqui-ai/TTS)
    ```bash
    # Install TTS
    python3 -m pip install TTS

    # Download models (when you call the tts command, they will be placed at ~/.local/share/tts/)
    cd /tmp
    tts --text "This command will download models" --model_name "tts_models/en/ljspeech/tacotron2-DDC" --vocoder_name "vocoder_models/en/ljspeech/hifigan_v2" --out_path speech.wav
    tts  --text "Isso vai baixar os modelos" --model_name tts_models/multilingual/multi-dataset/your_tts  --speaker_wav speech.wav --language_idx "pt-br"
    ```


        