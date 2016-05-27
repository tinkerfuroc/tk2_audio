#!/usr/bin/python

import xml.etree.ElementTree as ET
import subprocess

import rospy
from nltk.metrics import edit_distance
from std_msgs.msg import String
from termcolor import colored


class Answerer:
    def __init__(self, xml):
        self.question_dict = self.phrase_question(xml)

    @staticmethod
    def phrase_question(xml):
        xml_root = ET.parse(xml).getroot()
        questions = {}
        for question in xml_root.findall('question'):
            questions[question.find('q').text.upper()] = question.find('a').text
        return questions

    def answer_question(self, msg_question):
        msg_question = msg_question.data
        if len(msg_question) == 0:
            return
        rospy.loginfo(colored('[Q]%s', 'yellow'), msg_question)
        distance = {}
        for question in self.question_dict:
            distance[question] = edit_distance(question, msg_question)
        question = min(distance, key=distance.get)
        if distance[question] > 5:
            return
        answer = self.question_dict[question]
        rospy.loginfo(colored('[A]%s', 'green'), answer)
        self.speak(answer)

    @staticmethod
    def speak(answer):
        subprocess.Popen(('espeak', "'{}'".format(answer)))


def main():
    rospy.init_node('answer_question', anonymous=False)
    answerer = Answerer(rospy.get_param('~question_list'))
    rospy.loginfo(colored('starting answer question ...', 'green'))
    rospy.Subscriber('/recognizer/output', String, answerer.answer_question)
    rospy.spin()


if __name__ == '__main__':
    main()
