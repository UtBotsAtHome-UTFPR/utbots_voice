# utbots_voice: ROS packages of text-to-speech (TTS) and speech-to-text (STT)
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
    cd ~/catkin_ws/src
    git clone --recurse-submodules https://github.com/UtBotsAtHome-UTFPR/utbots_voice.git

    # Compile workspace
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```