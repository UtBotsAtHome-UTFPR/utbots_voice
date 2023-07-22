#!/usr/bin/env python3
import csv
import rospkg
import re
import rospy
from std_msgs.msg import String

class NLUnderstanding:
    def __init__(self):
        # Question-answers dict
        self.kword_dict = {}

        # Get path
        rp = rospkg.RosPack()
        package_path = rp.get_path('nlu_ros')

        # ROS init
        rospy.init_node("nlu_node", anonymous=True)

        # Get parameter

        keyword_databases = ["commands", "qa", "people", "drinks"]

        # /utbots/voice/nlu/param
        get_commands = rospy.get_param("/commands", True)
        get_qa = rospy.get_param("/qa", False)
        get_people = rospy.get_param("/people", False)
        get_drinks = rospy.get_param("/drinks", False)
        
        database_selection = {
        "commands": get_commands,
        "qa": get_qa,
        "people": get_people,
        "drinks": get_drinks
        }

        rospy.loginfo("[NLU] selected_databases:")
        for dbase in keyword_databases:
            file_path = package_path + "/assets/"
            if database_selection[dbase] == True:
                rospy.loginfo(dbase)
                file_path += dbase + ".csv"
                rospy.loginfo(f"Reading file: {file_path}")
                with open(file_path, 'r') as csvfile:
                    reader = csv.reader(csvfile, delimiter=';')
                    for row in reader:
                        self.kword_dict[row[0]] = row[1]

        # Subscriber
        rospy.Subscriber("/utbots/voice/stt/whispered", String, self.callback)

        # Publisher
        self.pub = rospy.Publisher("/utbots/voice/nlu", String, queue_size=10)
        self.pub_speech = rospy.Publisher("/utbots/voice/tts/robot_speech", String, queue_size=1)
        
        # Loop
        rospy.spin()

    # Receives command and returns answer
    def callback(self, msg):
        scores = []
        user_command = str(msg.data)
        for keywords, stored_command in self.kword_dict.items():                                          
            score = self.calculate_score(user_command, keywords)                                  
            scores.append((score, stored_command))                                           
        scores = sorted(scores, key=lambda x: x[0], reverse=True)
        if scores[0][0] <= 0.1:                                                                           
            rospy.loginfo("[NLU] Sorry, I did not understand you.") 
            self.pub_speech.publish("Sorry, please repeat.")
        else:
            command = scores[0][1]
            self.pub.publish(command)
            rospy.loginfo(f"[NLU] Understood: {command}") 

    # Calculates score of input command against stored command
    def calculate_score(self, input_command, stored_command):                                               
        input_words = set(re.findall('\w+', input_command.lower()))
        stored_words = set(re.findall('\w+', stored_command.lower()))                                                                                           
        common_words = input_words.intersection(stored_words)                                           
        score = 1.0 * len(common_words) / len(stored_words)                                                          
        return score

if __name__ == "__main__":
    NLUnderstanding()