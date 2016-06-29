#!/usr/bin/python

import rospkg
import rospy
import actionlib
import pyaudio

from threading import Thread
from pocketsphinx.pocketsphinx import *
from std_msgs.msg import String
from termcolor import colored

from tinker_audio_msgs.msg import RecognizeKeywordsAction, RecognizeKeywordsActionGoal
import socket


class RecognizeKeywordsActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('keywords_recognize', RecognizeKeywordsAction, self.execute, auto_start=False)
        self.server.start()


    def execute(self, goal):
        config = Decoder.default_config()
        config.set_string('-hmm', rospy.get_param('~hmm'))
        config.set_string('-dict', rospy.get_param('~dict'))
        config.set_string('-keyphrase', goal.keyword)
        config.set_float('-kws_threshold', 1e-15)
        config.set_float('-vad_threshold', 2.5)
        config.set_string('-logfn', '/dev/null')

        rospy.loginfo(colored('starting audio streaming ...', 'green'))
        audio_server = rospy.get_param('~audio_server', 'laptop.furoc.net')
        audio_port = rospy.get_param('~audio_port', 9012)
        audio_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        audio_s.connect((audio_server, audio_port))

        
        decoder = Decoder(config)
        decoder.start_utt()
        rospy.loginfo(colored('starting voice recognize ...', 'green'))

        while not rospy.is_shutdown():
            buf = audio_s.recv(1024)
            if buf:
                decoder.process_raw(buf, False, False)
            else:
                rospy.logerr("audio stream error")
                self.server.set_aborted()
                break
            if decoder.hyp() != None:
                rospy.loginfo(colored('keyword recognized', 'yellow'))
                self.server.set_succeeded()
                break

        rospy.loginfo(colored('stop recognize', 'yellow'))
        decoder.end_utt()
        audio_s.close()


def main():
    rospy.init_node('keywords_recognizer', anonymous=False)
    action_server = RecognizeKeywordsActionServer()
    rospy.spin()


if __name__ == '__main__':
    main()
