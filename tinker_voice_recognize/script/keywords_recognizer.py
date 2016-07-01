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
from audio_common_msgs.msg import AudioData
import socket
from threading import Lock


class RecognizeKeywordsActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('keywords_recognize', RecognizeKeywordsAction, self.execute, auto_start=False)
        self._l = Lock()
        self.buf = None
        self.server.start()


    def execute(self, goal):
        config = Decoder.default_config()
        config.set_string('-hmm', rospy.get_param('~hmm'))
        config.set_string('-dict', rospy.get_param('~dict'))
        config.set_string('-keyphrase', goal.keyword)
        config.set_float('-kws_threshold', 1e-20)
        config.set_float('-vad_threshold', 2.5)
        config.set_string('-logfn', '/dev/null')

        rospy.loginfo(colored('starting audio streaming ...', 'green'))
        
        
        decoder = Decoder(config)
        decoder.start_utt()
        rospy.loginfo(colored('starting voice recognize ...', 'green'))

        while not rospy.is_shutdown():
            with self._l:
                if self.buf:
                    decoder.process_raw(self.buf, False, False)
                    self.buf = None
                if decoder.hyp() != None:
                    rospy.loginfo(colored('keyword recognized', 'yellow'))
                    self.server.set_succeeded()
                    break

        rospy.loginfo(colored('stop recognize', 'yellow'))
        decoder.end_utt()
        audio_s.close()

    def record(self):
        audio_server = rospy.get_param('~audio_server', 'laptop.furoc.net')
        audio_port = rospy.get_param('~audio_port', 9012)
        audio_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        audio_s.connect((audio_server, audio_port))
        audio_pub = rospy.Publisher('/audio', AudioData, queue_size=10)
        while True:
            with self._l:
                buf = audio_s.recv(1024)
                audio_pub.publish(AudioData(buf))


def main():
    rospy.init_node('keywords_recognizer', anonymous=False)
    action_server = RecognizeKeywordsActionServer()
    action_server.record()
    rospy.spin()


if __name__ == '__main__':
    main()
