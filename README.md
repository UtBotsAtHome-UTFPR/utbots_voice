## ROS packages for handling human-robot interface
---
## Relevant information
- **For TTS, we offer two different solutions:**
    - **ROS wrapper for [Coqui TTS](https://github.com/coqui-ai/TTS)**
        - **Description**
            - Support to multiple languages, including Portuguese
    - **ROS wrapper for [Mimic TTS](https://github.com/MycroftAI/mimic3)**
        - **Description**
            - No Portuguese support
            - User friendly
    - Both Coqui and Mimic are good. One has Portuguese while the other doesn't.
- **For STT:**
    - **ROS wrapper for [Silero VAD](https://github.com/snakers4/silero-vad)**
        - **Description**
            - Continuously performs Voice Activity Detection (VAD) on your microphone
            - If it contains human voice, waits for a whole sentence to be completed
            - Then publishes "voiced audio" to a ROS topic
    - **ROS wrapper for [whisper.cpp](https://github.com/ggerganov/whisper.cpp)**
        - **Description**
            - whisper.cpp is a lightweight implementation of OpenAI's Whisper
            - Performs speech recogition
---
## Setup
- **Clone repository and compile workspace**
    ```bash
    # Clone repository
    cd ~/catkin_ws/src
    git clone --recurse-submodules https://github.com/UtBotsAtHome-UTFPR/utbots_voice.git

    # Compile workspace
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```
- **Setup packages**
    - Read steps in [utbots_stt](https://github.com/UtBotsAtHome-UTFPR/utbots_voice/blob/master/utbots_stt)
    - Read steps in [utbot_tts](https://github.com/UtBotsAtHome-UTFPR/utbots_voice/tree/master/utbots_tts)
---