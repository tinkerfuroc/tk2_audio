#!/usr/bin/python

import rospkg
import rospy
import pyaudio

from threading import Thread
from pocketsphinx.pocketsphinx import *
from std_msgs.msg import String
from tinker_audio_msgs.srv import *
from termcolor import colored
import socket


class Recognizer:
    def __init__(self):
        config = Decoder.default_config()
        config.set_string('-hmm', rospy.get_param('~hmm'))
        config.set_string('-lm', rospy.get_param('~lm'))
        config.set_string('-dict', rospy.get_param('~dict'))
        config.set_string('-fsg', rospy.get_param('~fsg'))
        config.set_string('-logfn', '/dev/null')
        self.config = config
        rospy.loginfo(colored('starting audio streaming ...', 'green'))
        audio_server = rospy.get_param('~audio_server', 'laptop.furoc.net')
        audio_port = rospy.get_param('~audio_port', 9012)
        self.audio_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.audio_s.connect((audio_server, audio_port))

    def worker(self):
        publisher = rospy.Publisher('/recognizer/output', String, queue_size=10)
        decoder = Decoder(self.config)
        decoder.start_utt()
        in_speech_bf = False
        rospy.loginfo(colored('starting voice recognize ...', 'green'))
        while not rospy.is_shutdown():
            buf = self.audio_s.recv(1024)
            if buf:
                decoder.process_raw(buf, False, False)
                if decoder.get_in_speech() != in_speech_bf:
                    in_speech_bf = decoder.get_in_speech()
                    if not in_speech_bf:
                        decoder.end_utt()
                        result = decoder.hyp().hypstr
                        publisher.publish(result)
                        rospy.loginfo(colored('Result:', 'green') + '%s', result)
                        decoder.start_utt()
            else:
                break
        decoder.end_utt()
        self.audio_s.close()


def main():
    rospy.init_node('voice_recognize', anonymous=False)
    recognizer = Recognizer()
    thread = Thread(target=recognizer.worker)
    thread.start()
    rospy.spin()
    thread.join()


if __name__ == '__main__':
    main()
