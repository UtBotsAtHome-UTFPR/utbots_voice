## Setup
```bash
# Download models
roscd whisper_cpp_ros
mkdir models/
cd models/
wget https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin -O ./ggml-base.en.bin # english only
wget https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.bin -O ./ggml-base.bin # works with multiple languages!

# Run VAD node
rosrun vad_silero_ros vad_node

# Run Whisper node
rosrun whisper_cpp_ros whisper_node
```
---
## Nodes
- **vad_node**
    - **File** 
        - ``vad_silero_ros/src/vad_node.cpp``
    - **Program description**
        - Uses Silero VAD model to perform voice activity detection

- **whisper_node**
    - **File** 
        - ``whisper_cpp_ros/src/whisper_node.cpp``
    - **Program description**
        - Uses whisper.cpp to perform speech recognition