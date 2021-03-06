#!/usr/bin/python

import xml.etree.ElementTree as ET
import subprocess

import rospy
import time
from nltk.metrics import edit_distance
from std_msgs.msg import String
from termcolor import colored

def build_word_keys(questions, prohibited_words):
    word_keys = {}
    key = 0
    for question in questions.keys():
        for word in question.split():
            if word not in word_keys and word not in prohibited_words:
                word_keys[word] = key
                key += 1
    return word_keys


def to_key_list(sentence, word_keys):
    return [word_keys[w] for w in sentence.split() if w in word_keys]


class Answerer:
    def __init__(self, xml):
        self.question_dict = self.phrase_question(xml)
        self.prohibited_words = {'where', 'are', 'what', 'of', 'or', 'is', 'in', 'which', 'who', 'the'}
        self.word_keys = build_word_keys(self.question_dict, self.prohibited_words)
        self.last_time = time.time()

    @staticmethod
    def phrase_question(xml):
        xml_root = ET.parse(xml).getroot()
        questions = {}
        for question in xml_root.findall('question'):
            questions[question.find('q').text.lower()] = question.find('a').text
        return questions

    def answer_question(self, msg_question):
        if time.time() - self.last_time < 3:
            return
        msg_question = msg_question.data.lower()
        print 'get' + msg_question
        if len(msg_question) == 0:
            return
        rospy.loginfo(colored('[Q]%s', 'yellow'), msg_question)
        distance = {}
        msg_keys = to_key_list(msg_question, self.word_keys)
        if len(msg_keys) == 0:
            return
        for question in self.question_dict:
            distance[question] = edit_distance(to_key_list(question, self.word_keys), 
                    to_key_list(msg_question, self.word_keys))
        question = min(distance, key=distance.get)
        question_keys = to_key_list(question, self.word_keys)
        if float(distance[question]) / float(len(question_keys)) > 0.8:
            return
        answer = self.question_dict[question]
        rospy.loginfo(colored('[A]%s', 'green'), answer)
        self.speak(answer)

    def speak(self, answer):
        subprocess.Popen(('espeak', "'{}'".format(answer)))
        self.last_time = time.time() + len(answer)//10
        rospy.sleep(len(answer) // 10)


def main():
    rospy.init_node('answer_question', anonymous=False)
    answerer = Answerer(rospy.get_param('~question_list'))
    rospy.loginfo(colored('starting answer question ...', 'green'))
    rospy.Subscriber('/recognizer/output', String, answerer.answer_question)
    answerer.speak('I am tinker, I am ready for answer questions')
    rospy.spin()


if __name__ == '__main__':
    main()
