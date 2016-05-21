#!/usr/bin/python
from os import path

import rospkg
import rospy
import pyaudio

from threading import Thread
from pocketsphinx.pocketsphinx import *
from std_msgs.msg import String
from termcolor import colored


def recognize():
    config = Decoder.default_config()
    config.set_string('-hmm', rospy.get_param('~hmm'))
    config.set_string('-lm', rospy.get_param('~lm'))
    config.set_string('-dict', rospy.get_param('~dict'))
    config.set_string('-logfn', '/dev/null')

    publisher = rospy.Publisher('/recognizer/output', String, queue_size=10)

    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
    stream.start_stream()

    decoder = Decoder(config)
    decoder.start_utt()
    in_speech_bf = False
    while not rospy.is_shutdown():
        buf = stream.read(1024)
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
    stream.stop_stream()


def main():
    rospy.init_node('voice_recognize', anonymous=False)
    rospy.loginfo(colored('starting voice recognize ...', 'green'))
    thread = Thread(target=recognize)
    thread.start()
    rospy.spin()
    thread.join()


if __name__ == '__main__':
    main()
