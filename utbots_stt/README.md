# utbots_stt

![alt text](image.png)

- **ROS wrapper for [Silero VAD](https://github.com/snakers4/silero-vad)**
  - Continuously performs Voice Activity Detection (VAD) on your microphone
  - If it contains human voice, waits for a whole sentence to be completed
  - Then publishes "voiced audio" to a ROS topic
  - Integration with [RNNoise](https://github.com/xiph/rnnoise) increases noise robustness
  - [Demonstration](https://www.youtube.com/watch?v=CYQ5u8lt4v8)

- **ROS wrapper for [Whisper (HF's Transformers)](  https://huggingface.co/openai/whisper-large-v3-turbo
)**
  - Heavyweight implementation of OpenAI's Whisper Turbo V3
  - Performs speech recognition
  - Synchronous transcription activated/deactivated with ROS services
  - Assynchronous single transcription with ROS actions
  - [Demonstration](https://www.youtube.com/watch?v=3EmWbu2jJg0)

![alt text](image-1.png)

## Installation

### Dependencies

If utbots_dependencies not already installed:

```bash
cd <ros2_ws>/src
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies.git
cd ../
```

#### Python
To avoid conflicts between package dependencies, we use virtual environments. Change the virtuelenv path in the `executable` field in `setup.cfg`. *Not the ideal solution, but the current one while we don't use Docker*.

If you haven't installed `virtualenv`:
```bash
pip3 install virtualenv
```

Create and activate env:
```bash
python -m virtualenv <env_path>
source <env_path>/bin/activate
```

#### Whisper
```bash
pip install --upgrade transformers accelerate
## Extra: 
pip install flash-attn --no-build-isolation ## May not work
```

#### Silero VAD
For noise suppression ([RNNoise](https://github.com/xiph/rnnoise)):
```bash
cd ~/
git clone https://github.com/xiph/rnnoise.git
cd rnnoise/
./autogen.sh
./configure
make
pip install git+https://github.com/Desklop/RNNoise_Wrapper.git
```

Installing Silero dependencies
```bash
pip install -r requirements.txt
sudo apt install portaudio19-dev
pip install pyaudio
```

## Building recomended:

```bash
cd <ros2_ws>
colcon build --packages-select vad_ros whisper_ros utbots_actions utbots_srvs utbots_msgs \
--allow-overriding utbots_msgs utbots_actions utbots_srvs \
&& source install/setup.bash
```

## Running

Run all STT launch [RNNoise + VAD + Whisper] (check for available parameters in the launch file):
```bash
ros2 launch vad_ros stt_launch.py
```

Run VAD node
```bash
ros2 run vad_ros vad_node
```

Run Whisper node
```bash
ros2 run whisper_ros whisper_full_node
# or
ros2 run whisper_ros whisper_node
```

## Parameters
```bash
ros2 param dump /vad_node 
# /vad_node:
#   ros__parameters:
#     use_sim_time: false
#     vad_threshold: 0.5
#     vad_timeout: 5000
#     vad_verbose: false


ros2 param dump /whisper_node #full
# /whisper_node:
#   ros__parameters:
#     enable_synchronous_startup: false
#     timer_period: 0.5
#     use_sim_time: false
#     wait_timeout: 12.0
#     whisper_model: openai/whisper-large-v3-turbo
#     whisper_startup: true
#     whisper_verbose: true
```


## Services
To toggle whisper sync transcription on/off:
```bash
ros2 service call /utbots/voice/enable_transcription std_srvs/srv/SetBool data:\ false\
```
Change the Whisper model (CURRENTLY NOT WORKING IN SHELL):
```bash
ros2 service call /whisper_model utbots_srvs/srv/SetString string:\ \ data:\ \'\'\
```

## Actions
To call the Transcription action server (12 seconds of timeout):
```bash
ros2 action send_goal /Transcription utbots_actions/action/Transcription {}\ 
```

#### TODO:
- Evaluate  **Distil-Whisper: Distil-Large-v3.5**
  https://huggingface.co/distil-whisper/distil-large-v3.5
