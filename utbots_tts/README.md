# utbots_tts

- **ROS wrapper for [Coqui TTS](https://github.com/coqui-ai/TTS)**
  - Synthesizes voice from text
  - 16 languages, including English and Portuguese
  - Assynchronous interface with ROS actions


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

Install requirements:
```bash
pip install -r requirements.txt
```

## Building recomended:

```bash
cd <ros2_ws>
colcon build --packages-select ros_tts utbots_actions utbots_srvs utbots_msgs \
--allow-overriding utbots_msgs utbots_actions utbots_srvs \
&& source install/setup.bash
```

## Running

Run all STT launch [RNNoise + VAD + Whisper] (check for available parameters in the launch file):
```bash
ros2 launch ros_tts tts_launch.py
```

Run TTS node
```bash
ros2 run ros_tts tts_node
```

## Parameters
- model_name: model path string
- use_cuda: true or false
- verbose: true or false

## Actions
To call the Transcription action server (12 seconds of timeout):
```bash
ros2 action send_goal /utbots/tts utbots_actions/action/TextToSpeech "{text: <your text>}" -f
```
