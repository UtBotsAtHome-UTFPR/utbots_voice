## ROS packages for handling human-robot interface
- [Demonstration](https://www.youtube.com/watch?v=4TaugaMfJ-8)
---
## Relevant information
- **For TTS, we offer two different solutions:**
    - **ROS wrapper for [Coqui TTS](https://github.com/coqui-ai/TTS)**
        - Support to multiple languages, including Portuguese
        - [Demonstration](https://www.youtube.com/watch?v=Lzg7fp4lqDg)
    - **ROS wrapper for [Mimic TTS](https://github.com/MycroftAI/mimic3)**
        - No Portuguese support
        - User friendly
        - [Demonstration](https://www.youtube.com/watch?v=mtwtwYdP4dc)
    - Both Coqui and Mimic are good
- **For STT:**
    - **ROS wrapper for [Silero VAD](https://github.com/snakers4/silero-vad)**
        - Continuously performs Voice Activity Detection (VAD) on your microphone
        - If it contains human voice, waits for a whole sentence to be completed
        - Then publishes "voiced audio" to a ROS topic
        - [Demonstration](https://www.youtube.com/watch?v=CYQ5u8lt4v8)
    - **ROS wrapper for [whisper.cpp](https://github.com/ggerganov/whisper.cpp)**
        - whisper.cpp is a lightweight implementation of OpenAI's Whisper
        - Performs speech recogition
        - [Demonstration](https://www.youtube.com/watch?v=3EmWbu2jJg0)
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