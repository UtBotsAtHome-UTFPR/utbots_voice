# whisper_cpp_ros: ROS package for whisper.cpp

- ### Setup package
    ```bash
    # Clone repository
    cd ~catkin_ws/src
    git clone --recurse-submodules https://github.com/UtBotsAtHome-UTFPR/whisper_cpp_ros.git

    # Compile
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash

    # Download models
    roscd whisper_cpp_ros
    mkdir models/
    cd models/
    wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin -O ./ggml-base.en.bin # english only
    wget https://huggingface.co/datasets/ggerganov/whisper.cpp/resolve/main/ggml-base.bin -O ./ggml-base.bin # works with multiple languages!
    ```

- ### Nodes
    - #### vad.py
        - Program description
            - Takes input audio frames through ROS
            - Performs Voice Activity Detection using Silero VAD (https://github.com/snakers4/silero-vad)
            - Publishes voice clip
        - Subscribers
            - /audio (``audio_common_msgs/AudioData``)
                - Designed to receive audio from microphone, tipically with audio_capture (http://wiki.ros.org/audio_capture)
            - /audio_info (``audio_common_msgs/AudioInfo``)
                - Gets audio info (sample rate, channels, etc)
        - Publishers
            - /audio/voice (``audio_common_msgs/AudioData``)
                - Publishes audio clip containing voice
        - How to run
            ```bash
            roslaunch whisper_cpp_ros audio_capture.launch
            roslaunch whisper_cpp_ros vad.launch
            ```

    - #### whisper_node.cpp
        - Program description
            - Uses whisper.cpp to perform speech recognition
        - Subscribers
            - /audio/voice (``audio_common_msgs/AudioData``)
                - Gets wave audio and then runs the whisper.cpp model for it
        - How to run
            ```bash
            roslaunch whisper_cpp_ros whisper.launch
            ```