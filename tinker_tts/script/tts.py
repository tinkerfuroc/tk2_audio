#!/usr/bin/python

import rospy
import subprocess

from std_msgs.msg import String


def speak(text):
    rospy.logdebug('speak %s', text)
    subprocess.Popen(('espeak', "'{}'".format(text)))

def main():
    rospy.init_node('tinker_tts', anonymous=False)
    rospy.loginfo('starting tts ...')
    rospy.Subscriber('tts', String, speak)
    rospy.spin()

if __name__ == '__main__':
    main()
