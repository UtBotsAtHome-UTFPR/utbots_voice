## Setup
```bash
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
    - **How to run**
        - ```rosrun vad_silero_ros vad_node```

- **whisper_node**
    - **File** 
        - ``whisper_cpp_ros/src/whisper_node.cpp``
    - **Program description**
        - Uses whisper.cpp to perform speech recognition
    - **How to run**
        - ```rosrun whisper_cpp_ros whisper_node```