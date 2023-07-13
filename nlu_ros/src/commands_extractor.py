#!/usr/bin/env python3
import csv
import rospkg
import re
import rospy
from std_msgs.msg import String

class CommandsExtractor:
    def __init__(self):
        # Question-answers dict
        self.comm_dict = {}

        # Get path
        rp = rospkg.RosPack()
        package_path = rp.get_path('nlu_ros')
        file_path = package_path + "/assets/stored_commands.csv"

        # ROS init
        rospy.init_node("commands_extractor", anonymous=True)

        # Get parameter
        param_csv = rospy.get_param("~csv")
        rospy.loginfo(f"[COMM_EXTRACTOR] param_csv: {param_csv}")
        file_path = package_path + "/assets/" + param_csv

        # Subscriber
        rospy.Subscriber("/utbots/voice/stt/whispered", String, self.callback)

        # Publisher
        self.pub = rospy.Publisher("/utbots/task_manager/voice_command", String, queue_size=10)
        self.pub_speech = rospy.Publisher("/utbots/voice/tts/robot_speech", String, queue_size=1)

        # Process QuestionsAnswers.csv file
        rospy.loginfo(f"[COMM_EXTRACTOR] Reading file: {file_path}")
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile, delimiter=';')
            for row in reader:
                self.comm_dict[row[0]] = row[1]
        
        # Loop
        rospy.spin()

    # Receives command and returns answer
    def callback(self, msg):
        scores = []
        user_command = str(msg.data)
        for keywords, stored_command in self.comm_dict.items():                                          
            score = self.calculate_score(user_command, keywords)                                  
            scores.append((score, stored_command))                                           
        scores = sorted(scores, key=lambda x: x[0], reverse=True)
        if scores[0][0] <= 0.1:                                                                           
            rospy.loginfo("[COMM_EXTRACTOR] Sorry, I don't know that command.") 
            self.pub_speech.publish("Sorry, please repeat.")
        else:
            command = scores[0][1]
            self.pub.publish(command)
            rospy.loginfo(f"[COMM_EXTRACTOR] Command: {command}") 

    # Calculates score of input command against stored command
    def calculate_score(self, input_command, stored_command):                                               
        input_words = set(re.findall('\w+', input_command.lower()))
        stored_words = set(re.findall('\w+', stored_command.lower()))                                                                                           
        common_words = input_words.intersection(stored_words)                                           
        score = 1.0 * len(common_words) / len(stored_words)                                                          
        return score

if __name__ == "__main__":
    CommandsExtractor()