#!/usr/bin/python

import xml.etree.ElementTree as ET
import subprocess

import rospy
from nltk.metrics import edit_distance
from std_msgs.msg import String
from termcolor import colored

def build_word_keys(questions, prohibited_words):
    word_keys = {}
    key = 0
    for question in questions.keys():
        for word in question.split():
            if word not in word_keys and word not in prohibited_words:
                word_keys[word] = keys
                keys += 1
    return word_keys


def to_key_list(sentence, word_keys):
    return [word_keys[w] for w in sentence.split() if w in word_keys]


class Answerer:
    def __init__(self, xml):
        self.question_dict = self.phrase_question(xml)
        self.prohibited_words = {'where', 'are', 'what', 'of',
                'or', 'is', 'in', 'which', 'who', 'the'}
        self.word_keys = build_word_keys(questions, self.prohibited_words)

    @staticmethod
    def phrase_question(xml):
        xml_root = ET.parse(xml).getroot()
        questions = {}
        for question in xml_root.findall('question'):
            questions[question.find('q').text.lower()] = question.find('a').text
        return questions

    def answer_question(self, msg_question):
        msg_question = msg_question.data.lower()
        if len(msg_question) == 0:
            return
        rospy.loginfo(colored('[Q]%s', 'yellow'), msg_question)
        distance = {}
        for question in self.question_dict:
            distance[question] = edit_distance(to_key_list(question), to_key_list(msg_question))
        question = min(distance, key=distance.get)
        if distance[question] > 10:
            return
        answer = self.question_dict[question]
        rospy.loginfo(colored('[A]%s', 'green'), answer)
        self.speak(answer)

    @staticmethod
    def speak(answer):
        subprocess.Popen(('espeak', "'{}'".format(answer)))
        rospy.sleep(5)


def main():
    rospy.init_node('answer_question', anonymous=False)
    answerer = Answerer(rospy.get_param('~question_list'))
    rospy.loginfo(colored('starting answer question ...', 'green'))
    rospy.Subscriber('/recognizer/output', String, answerer.answer_question)
    rospy.spin()


if __name__ == '__main__':
    main()
