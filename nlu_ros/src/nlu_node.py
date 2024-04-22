#!/usr/bin/env python3
import csv
import rospkg
import re
import rospy
import time
from std_msgs.msg import String
from voice_msgs.msg import NLU

class NLUnderstanding:
    def __init__(self):
        # Question-answers dict
        self.kword_dict = {}

        # Get path
        rp = rospkg.RosPack()
        self.package_path = rp.get_path('nlu_ros')

        # ROS init
        rospy.init_node("nlu_node", anonymous=True)

        # Flags
        self.firstMessage = True

        # Subscriber
        rospy.Subscriber("/utbots/voice/stt/whispered", String, self.callback)

        # Publisher
        self.pub_nlu = rospy.Publisher("/utbots/voice/nlu", String, queue_size=10)
        self.pub_nlumsg = rospy.Publisher("/utbots/voice/nlu_msg", NLU, queue_size=10)
        self.pub_speech = rospy.Publisher("/utbots/voice/tts/robot_speech", String, queue_size=1)
        
        # Get parameter

        self.keyword_databases = ["commands", "qa", "people", "drinks"]

        # Loop
        rospy.spin()

    # Receives command and returns answer
    def callback(self, msg):

        if self.firstMessage:
            self.firstMessage = False
            # /utbots/voice/nlu/param
            get_commands = rospy.get_param("/voice/nlu_node/commands", True)
            get_qa = rospy.get_param("/voice/nlu_node/qa", False)
            get_people = rospy.get_param("/voice/nlu_node/people", False)
            print(get_people)
            get_drinks = rospy.get_param("/voice/nlu_node/drinks", False)
            
            database_selection = {
            "commands": get_commands,
            "qa": get_qa,
            "people": get_people,
            "drinks": get_drinks
            }

            rospy.loginfo("[NLU] selected_databases:")
            for dbase in self.keyword_databases:
                file_path = self.package_path + "/assets/"
                if database_selection[dbase] == True:
                    rospy.loginfo(dbase)
                    file_path += dbase + ".csv"
                    rospy.loginfo(f"Reading file: {file_path}")
                    with open(file_path, 'r') as csvfile:
                        reader = csv.reader(csvfile, delimiter=';')
                        for row in reader:
                            self.kword_dict[row[0]] = [row[1], dbase]
                            rospy.loginfo(f"[NLU] {row[0], row[1], dbase}") 
        
        scores = []
        user_command = str(msg.data)
        for keywords, stored_command in self.kword_dict.items():                                          
            score = self.calculate_score(user_command, keywords)                                  
            scores.append((score, stored_command[0], stored_command[1]))                                        
        scores = sorted(scores, key=lambda x: x[0], reverse=True)
        if scores[0][0] <= 0.1:                                                                           
            rospy.loginfo("[NLU] Sorry, I did not understand you.") 
            self.pub_speech.publish("Sorry, please repeat.")
        else:
            command = scores[0][1]
            dbase = scores[0][2]
            self.pub_speech.publish(command)
            self.pub_nlu.publish(command)
            self.pub_nlumsg.publish(NLU(String(command), String(dbase)))
            rospy.loginfo(f"[NLU] Understood {command} from {dbase}") 

    # Calculates score of input command against stored command
    def calculate_score(self, input_command, stored_command):                                               
        input_words = set(re.findall('\w+', input_command.lower()))
        stored_words = set(re.findall('\w+', stored_command.lower()))                                                                                           
        common_words = input_words.intersection(stored_words)                                           
        score = 1.0 * len(common_words) / len(stored_words)                                                          
        return score

if __name__ == "__main__":
    NLUnderstanding()
