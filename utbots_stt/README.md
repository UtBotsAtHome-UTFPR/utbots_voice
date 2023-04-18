## Setup package
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
---

## Nodes
- **vad_node**
    - **File** 
        - ``vad_silero_ros/src/vad_node.cpp``
    - **Program description**
        - Uses Silero VAD model to perform voice activity detection
    - **Subscribers**
        - ...
    - **Publishers**
        - ...
    - **How to run**
        - ```rosrun vad_silero_ros vad_node```

- **whisper_node**
    - **File:** ``whisper_cpp_ros/src/whisper_node.cpp``
    - **Program description**
        - Uses whisper.cpp to perform speech recognition
    - **Subscribers**
        - ...
    - **Publishers**
        - ...
    - **How to run**
        - ```rosrun whisper_cpp_ros whisper_node```