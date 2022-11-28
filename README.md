# apollo_voice: pacotes ROS de text-to-speech (TTS) e speech-to-text (STT)
- Para TTS, usamos Coqui TTS (sucessor do Mozilla TTS)
- Para STT, usamos whisper.cpp (implementação de alta performance do OpenAI Whisper)
- Portanto, os pacotes desse repositório apenas implementam uma interface via ROS para os programas referidos

## Setup do pacote
- ### Clonar repositório do pacote
    - ```git clone https://github.com/UtBot-UTFPR/apollo_voice.git```
- ### TTS
    - Instalar o TTS
        - TODO
    - Baixar modelos
        - TODO
- ### STT
    - Clonar repositório do whisper.cpp e compilar
        - ```cd ~/catkin_ws/apollo_voice```
        - ```git clone https://github.com/ggerganov/whisper.cpp.git```
        - ```cd whisper.cpp && make```
    - Baixar modelos
        - ```cd ~/catkin_ws/apollo_voice```
        - ```wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin -O resources/models/ggml-base.en.bin```
        - ```wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.bin -O resources/models/ggml-base.bin```
    - Instalar dependências de processamento de áudio
        - ```python3 -m pip install soundfile librosa noisereduce```
    - Executar o programa
        - ```rosrun apollo_voice ros_stt.py```