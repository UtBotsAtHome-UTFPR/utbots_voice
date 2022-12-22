#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from os import system

class SpeechSynthesisNode:

    # Init
    def __init__(self):
        rospy.init_node('tts', anonymous=True)
        rospy.loginfo("[TTS] Mimic TTS node init")
        self.voice = "--voice en_US/hifi-tts_low"
        self.sub_text = rospy.Subscriber("/voice/tts/text", String, self.Callback)
        self.loopRate = rospy.Rate(30)
        self.Say("Hello! I am Apollo, your service robot. Call me if you need anything.")
        self.MainLoop()

    # Callback for received text
    def Callback(self, msg):
        rospy.loginfo("[TTS] Callback: text is '{}'".format(msg.data))
        self.Say(msg.data)

    # Converts text input to audio output
    def Say(self, text):
        text = text.replace("'", "").replace("\"", "")
        command = "mimic3 {} '{}' | aplay".format(self.voice, text)
        rospy.loginfo("[TTS] System call: {}".format(command))
        system(command)
        rospy.loginfo("[TTS] Played audio")

    # Main loop
    def MainLoop(self):
        rospy.loginfo("[TTS] Looping...")
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()

if __name__ == "__main__":
    SpeechSynthesisNode()