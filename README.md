# utbots_voice

This stack contains text-to-speech, speech-to-text and face emotions packages.

## Installation
```git clone --recurse-submodules https://github.com/UtBot-UTFPR/apollo_voice.git```

## Packages

### voztts (text-to-speech)

**Dependencies**

This package depends on docker and MaryTTS API for text-to-speech, running as a docker container.

```
sudo apt install docker mplayer
sudo docker pull synesthesiam/marytts:5.2
```

**Running**

First, run MaryTTS as a docker container

```sudo docker run -it -p 59125:59125 synesthesiam/marytts:5.2 --voice cmu-bdl-hsmm```

You can check if the server is available

```http://localhost:59125/```

Then, run the launchfile (system password is required)

```roslaunch voztts vozttp.launch autorestart:=true```

**Test commands**

```rostopic pub /emotion std_msgs/String "joy"```

```rostopic pub /tts std_msgs/String "HELLO WORLD"```

```rostopic pub /emotion std_msgs/String "rage"```

```rostopic pub /tts std_msgs/String "HELLO WORLD"```

**Nodes**

Subscribers

- /tts (std_msgs/String)
Text to be synthesized
- /emotion (std_msgs/String)
Changes the voice tone

Possible emotions
- "rage"
- "annoyance"
- "anger"
- "idle"
- "terror"
- "fear"
- "apprehension"
- "ecstasy"
- "joy"
- "serenity"
- "grief"
- "sadness"
- "pensiveness"
- "vigilance"
- "antecipation"
- "interest"
- "loathing"
- "disgust"
- "boredom"
- "amazement"
- "surprise"
- "distraction"
- "admiration"
- "trust"
- "acceptance"

## utbots_at_home_voice (para speech-to-text)

**Dependencies**
An Android device is required with the ROS Voice Recognition apk installed.

Install the needed python libraries with the following command

```python3 -m pip install nltk sklearn pandas gensim```

**Running**

```roslaunch utbots_at_home_voice utbots_at_home_voicerecog.launch``` For the question and answers task

```roslaunch utbots_at_home_voice commands.launch``` For listening to voice commands

**Testing**
Talk to the Android device and run

```rostopic echo /text_recognized```

**Nodes**

- subscreve_node (src/subscribe.cpp)
    - Subscribers:
        - /Tablet/voice (std_msgs/String)
- pubsub_node (src/publish.cpp)
    - Publishers:
        - /text_recognized (std_msgs/String)
    - Subscribers:
        - /Tablet/voice (std_msgs/String)
- listen_node (scripts/listen.py)
    - Subscribers:
        - /text_recognized (std_msgs/String)

### ros_display_emotions
ROS package that displays emotions through faces. Working in Kinetic and Melodic.

Please install the following dependencies before running the package:

```sudo apt-get install espeak espeak-data libespeak-dev```
```sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev```

*The full list of emotions accepted is:*

annoyance, much_annoyance, anger, rage, interest, much_interest, anticipation, vigilance, boredom, much_boredom, disgust, loathing, apprehension, much_apprehension, fear, terror, serenity, much_serenity, joy, ecstasy, pensiveness, much_pensiveness, sadness, grief, distraction, much_distraction, surprise, amazement, acceptance, much_acceptance, trust, admiration, idle

*To display an emotion, first run the launch file*

```roslaunch display_emotions display_emotions.launch```

*And then publish a message of type String to the topic /emotion*

```rostopic pub /emotion std_msgs/String "data: 'joy'"```

*Parameters*

faces_cycle: make the images cycle between the most intense emotion desired or stay still at the emotion set (default: true)

faces_cycle_delay: the delay in seconds to shift between emotions (only active if faces_cycle is active)
        
        
