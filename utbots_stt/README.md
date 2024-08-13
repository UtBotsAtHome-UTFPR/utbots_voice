# utbots_stt

- **ROS wrapper for [Silero VAD](https://github.com/snakers4/silero-vad)**
  - Continuously performs Voice Activity Detection (VAD) on your microphone
  - If it contains human voice, waits for a whole sentence to be completed
  - Then publishes "voiced audio" to a ROS topic
  - [Demonstration](https://www.youtube.com/watch?v=CYQ5u8lt4v8)

- **ROS wrapper for [whisper.cpp](https://github.com/ggerganov/whisper.cpp)**
  - whisper.cpp is a lightweight implementation of OpenAI's Whisper
  - Performs speech recogition
  - [Demonstration](https://www.youtube.com/watch?v=3EmWbu2jJg0)

## Installation

### Dependencies

```bash
# Download models
roscd whisper_cpp_ros
mkdir models/
cd models/
wget https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-tiny.en.bin -O ./ggml-tiny.en.bin # tiny model english only
wget https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin -O ./ggml-base.en.bin # english only
wget https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.bin -O ./ggml-base.bin # works with multiple languages!
```

## Running

Run VAD node
```bash
rosrun vad_silero_ros vad_node
```

Run Whisper node
```bash
rosrun whisper_cpp_ros whisper_node
```

## Nodes

### vad_silero_ros

**File** 

``vad_silero_ros/src/vad_node.cpp``

**Program description**

Uses Silero VAD model to perform voice activity detection

### whisper_cpp_ros

**File** 

``whisper_cpp_ros/src/whisper_node.cpp``

**Program description**

Uses whisper.cpp to perform speech recognition
  
