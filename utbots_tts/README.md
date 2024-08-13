# utbots_tts

- **ROS wrapper for [Coqui TTS](https://github.com/coqui-ai/TTS)**
  - Support to multiple languages, including Portuguese
  - [Demonstration](https://www.youtube.com/watch?v=Lzg7fp4lqDg)

- **ROS wrapper for [Mimic TTS](https://github.com/MycroftAI/mimic3)**
  - No Portuguese support
  - User friendly
  - [Demonstration](https://www.youtube.com/watch?v=mtwtwYdP4dc)
  - 
- Both Coqui and Mimic are good, Mimic is used as default

## Installation

### [Mimic3 TTS](https://github.com/MycroftAI/mimic3) Dependencies
  
```bash
# Download mimic3
cd ~/Downloads
wget https://github.com/MycroftAI/mimic3/releases/download/release%2Fv0.2.4/mycroft-mimic3-tts_0.2.4_amd64.deb

# Install with apt
sudo apt install ./mycroft-mimic3-tts_0.2.4_amd64.deb
```

### [Coqui TTS](https://github.com/coqui-ai/TTS) Dependencies
  
```bash
# Install TTS
python3 -m pip install TTS

# Install Python dependencies
python3 -m pip install playsound

# Download models (they will be placed at ~/.local/share/tts/)
cd /tmp
tts --text "This command will download models" --model_name "tts_models/en/ljspeech/tacotron2-DDC" --vocoder_name "vocoder_models/en/ljspeech/hifigan_v2" --out_path speech.wav
tts  --text "Isso vai baixar os modelos" --model_name tts_models/multilingual/multi-dataset/your_tts  --speaker_wav speech.wav --language_idx "pt-br"
```

## Running

### Mimic3

You can test Mimic3 separately (it should take a small time to download the model):
```bash
mimic3 --voice en_US/hifi-tts_low 'Hello world' | aplay
```

To launch the Mimic3 ROS node:
```bash
roslaunch utbots_tts tts_mimic.launch
```

You can test the audio output by sending a text to a topic in another terminal:
```bash
rostopic pub /robot_speech std_msgs/String 'Hello world'
```

### Coqui TTS

To launch the Coqui TTS ROS node:
```bash
roslaunch utbots_tts tts_coqui.launch
```

You can test the audio output by sending a text to a topic in another terminal:
```bash
rostopic pub /robot_speech std_msgs/String 'Olá mundo!'
```
    ```
