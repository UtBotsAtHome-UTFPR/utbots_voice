<<<<<<< HEAD
# utbots_voice

This stack contains text-to-speech, speech-to-text and face emotions packages.

## Installation
```git clone --recurse-submodules https://github.com/UtBot-UTFPR/apollo_voice.git```

## Packages

### voztts (text-to-speech)

**Dependencies**

This package depends on docker and MaryTTS API for text-to-speech, running as a docker container.

```
sudo apt install docker mplayer
sudo docker pull synesthesiam/marytts:5.2
```

**Running**

First, run MaryTTS as a docker container

```sudo docker run -it -p 59125:59125 synesthesiam/marytts:5.2 --voice cmu-bdl-hsmm```

You can check if the server is available

```http://localhost:59125/```

Then, run the launchfile (system password is required)

```roslaunch voztts vozttp.launch autorestart:=true```

**Test commands**

```rostopic pub /emotion std_msgs/String "joy"```

```rostopic pub /tts std_msgs/String "HELLO WORLD"```

```rostopic pub /emotion std_msgs/String "rage"```

```rostopic pub /tts std_msgs/String "HELLO WORLD"```

**Nodes**

Subscribers

- /tts (std_msgs/String)
Text to be synthesized
- /emotion (std_msgs/String)
Changes the voice tone

Possible emotions
- "rage"
- "annoyance"
- "anger"
- "idle"
- "terror"
- "fear"
- "apprehension"
- "ecstasy"
- "joy"
- "serenity"
- "grief"
- "sadness"
- "pensiveness"
- "vigilance"
- "antecipation"
- "interest"
- "loathing"
- "disgust"
- "boredom"
- "amazement"
- "surprise"
- "distraction"
- "admiration"
- "trust"
- "acceptance"

## utbots_at_home_voice (para speech-to-text)

**Dependencies**
An Android device is required with the ROS Voice Recognition apk installed.

Install the needed python libraries with the following command

```python3 -m pip install nltk sklearn pandas gensim```

**Running**

```roslaunch utbots_at_home_voice utbots_at_home_voicerecog.launch``` For the question and answers task

```roslaunch utbots_at_home_voice commands.launch``` For listening to voice commands

**Testing**
Talk to the Android device and run

```rostopic echo /text_recognized```

**Nodes**

- subscreve_node (src/subscribe.cpp)
    - Subscribers:
        - /Tablet/voice (std_msgs/String)
- pubsub_node (src/publish.cpp)
    - Publishers:
        - /text_recognized (std_msgs/String)
    - Subscribers:
        - /Tablet/voice (std_msgs/String)
- listen_node (scripts/listen.py)
    - Subscribers:
        - /text_recognized (std_msgs/String)

### ros_display_emotions

ROS package that displays emotions through faces. Working in Kinetic and Melodic.

Please install the following dependencies before running the package:

```sudo apt-get install espeak espeak-data libespeak-dev```
```sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev```

**The full list of emotions accepted is:**

annoyance, much_annoyance, anger, rage, interest, much_interest, anticipation, vigilance, boredom, much_boredom, disgust, loathing, apprehension, much_apprehension, fear, terror, serenity, much_serenity, joy, ecstasy, pensiveness, much_pensiveness, sadness, grief, distraction, much_distraction, surprise, amazement, acceptance, much_acceptance, trust, admiration, idle

**To display an emotion, first run the launch file**

```roslaunch display_emotions display_emotions.launch```

**And then publish a message of type String to the topic /emotion**

```rostopic pub /emotion std_msgs/String "data: 'joy'"```

**Parameters**

faces_cycle: make the images cycle between the most intense emotion desired or stay still at the emotion set (default: true)

faces_cycle_delay: the delay in seconds to shift between emotions (only active if faces_cycle is active)
        
        
=======
# utbots_voice: ROS package of text-to-speech (TTS) and speech-to-text (STT)
- For TTS:
    - We have a ROS interface of Coqui TTS (Mozilla TTS successor, bleeding edge TTS that supports a variety of models).
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
    wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin -O ./ggml-base.en.bin # english only
    wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.bin -O ./ggml-base.bin # works with multiple languages

    # Install Python dependencies
    python3 -m pip install soundfile librosa noisereduce

    # Run the test program (it tries to transcript wav from resources/wav/samples/baka_gaijin.wav)
    rosrun utbots_voice ros_stt.py
    ```
- ### Setup Mimic3 TTS (https://github.com/MycroftAI/mimic3)
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

    # Install Python dependencies
    python3 -m pip install playsound

    # Download models (they will be placed at ~/.local/share/tts/)
    cd /tmp
    tts --text "This command will download models" --model_name "tts_models/en/ljspeech/tacotron2-DDC" --vocoder_name "vocoder_models/en/ljspeech/hifigan_v2" --out_path speech.wav
    tts  --text "Isso vai baixar os modelos" --model_name tts_models/multilingual/multi-dataset/your_tts  --speaker_wav speech.wav --language_idx "pt-br"

    # Test the Coqui TTS ROS node
    rosrun utbots_voice ros_tts_coqui.py
    rostopic pub /voice/tts/text std_msgs/String 'Olá mundo!' # (in another terminal)
    ```

    
>>>>>>> steroids
