#!/usr/bin/env python3
import csv
import rospkg
import re
import rospy
from std_msgs.msg import String

class QuestionAnswererNode:
    def __init__(self):
        # Question-answers dict
        self.qa_dict = {}

        # Get path
        rp = rospkg.RosPack()
        package_path = rp.get_path('nlu_ros')
        file_path = package_path + "/assets/questions-robocup-brazil-2022.csv"

        # ROS init
        rospy.init_node("question_answerer_node", anonymous=True)

        # Get parameter
        param_csv = rospy.get_param("~csv")
        rospy.loginfo(f"[QA] param_csv: {param_csv}")
        file_path = package_path + "/assets/" + param_csv

        # Subscriber
        rospy.Subscriber("question", String, self.callback)

        # Publisher
        self.pub = rospy.Publisher('answer', String, queue_size=10)

        # Process QuestionsAnswers.csv file
        rospy.loginfo(f"[QA] Reading file: {file_path}")
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile, delimiter=';')
            for row in reader:
                self.qa_dict[row[0]] = row[1]
        
        # Loop
        rospy.spin()

    # Receives question and returns answer
    def callback(self, msg):
        scores = []
        user_question = str(msg.data)
        for stored_question, stored_answer in self.qa_dict.items():                                          
            score = self.calculate_score(user_question, stored_question)                                     
            scores.append((score, stored_answer))                                                       
        scores = sorted(scores, key=lambda x: x[0], reverse=True)
        if scores[0][0] <= 0.1:                                                                           
            rospy.loginfo("[QA] Sorry, I don't know the answer to that question.")                                   
        else:
            answer = scores[0][1]
            self.pub.publish(answer)
            rospy.loginfo(f"[QA] Answer: {answer}") 

    # Calculates score of input question against stored question
    def calculate_score(self, input_question, stored_question):                                               
        input_words = set(re.findall('\w+', input_question.lower()))
        stored_words = set(re.findall('\w+', stored_question.lower()))                                                                                           
        common_words = input_words.intersection(stored_words)                                           
        score = 1.0 * len(common_words) / len(stored_words)                                                          
        return score

if __name__ == "__main__":
    QuestionAnswererNode()