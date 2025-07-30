# utbots_voice

This stack contains packages related to human-robot interface, such as:

- [display_emotions](https://github.com/UtBotsAtHome-UTFPR/display_emotions/tree/574f91eab071ab4ef88e66dae85b1703996774e3)
- **utbots_stt**
- **utbots_tts**
- **[utbots_nlu](https://github.com/UtBotsAtHome-UTFPR/utbots_nlu)**

And is dependant on:

- [utbots_dependencies](https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies)

## Installation

```bash 
cd <ros2_ws>/src
git clone --recurse-submodules https://github.com/UtBotsAtHome-UTFPR/utbots_voice.git
cd ../
```

### Dependencies

See the dependencies installation procedure for each package accessing its README.md file or below, in [Packages Description](#packages-description).

### Building

```bash
cd <ros2_ws>
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select display_emotions utbots_nlu ros_tts vad_ros whisper_ros utbots_actions utbots_srvs utbots_msgs \
--allow-overriding utbots_msgs utbots_actions utbots_srvs \
&& source install/setup.bash
```

### Updating

To push changes to the submodule packages ([display_emotions](https://github.com/UtBotsAtHome-UTFPR/display_emotions/tree/574f91eab071ab4ef88e66dae85b1703996774e3), [utbots_nlu](https://github.com/UtBotsAtHome-UTFPR/utbots_nlu)) you should go to their repository path and perform a simple add, commit and push. After, you have to push the changes to the stack, going back to the stack repository path and doing the following command:

```bash
git submodule update --remote --merge
```
And then, perform a simple add, commit and push in the stack repository.

## Running

See the usage explanation accessing each package in the repository or below, in [Packages Description](#packages-description).

## Packages Description

See the README.md in each package.
