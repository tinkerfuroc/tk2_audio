#!/usr/bin/python

import rospkg
import rospy
import actionlib
import pyaudio

from threading import Thread
from pocketsphinx.pocketsphinx import *
from std_msgs.msg import String
from termcolor import colored

from tinker_audio_msgs.msg import RecognizeKeywordsAction


class RecognizeKeywordsActionServer:
    def __init__(self):
        audio = pyaudio.PyAudio()
        self.stream = audio.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)

        self.server = actionlib.SimpleActionServer('keywords_recognize', RecognizeKeywordsAction, self.execute, auto_start=False)
        self.server.start()


    def execute(self, goal):
        config = Decoder.default_config()
        config.set_string('-hmm', rospy.get_param('~hmm'))
        config.set_string('-dict', rospy.get_param('~dict'))
        config.set_string('-keyphrase', goal.keyword)
        config.set_float('-kws_threshold', 1e-4)
        config.set_string('-logfn', '/dev/null')

        self.stream.start_stream()
        rospy.loginfo(colored('starting audio streaming ...', 'green'))
        
        decoder = Decoder(config)
        decoder.start_utt()
        rospy.loginfo(colored('starting voice recognize ...', 'green'))

        while not rospy.is_shutdown():
	    buf = self.stream.read(1024)
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
                
        decoder.end_utt()
        self.stream.stop_stream()


def main():
    rospy.init_node('keywords_recognizer', anonymous=False)
    action_server = RecognizeKeywordsActionServer()
    rospy.spin()


if __name__ == '__main__':
    main()
