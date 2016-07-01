#!/usr/bin/python

import rospy
import roslib
import actionlib
import sys
from tinker_msgs.msg._SimpleMoveGoal import SimpleMoveGoal
from tinker_msgs.msg._SimpleMoveAction import SimpleMoveAction
from std_msgs.msg import Float64
import socket
from struct import unpack
from math import fabs

threshold = 0.1 # 5 degree

def read_latest(s, l):
    buf = None
    while not rospy.is_shutdown():
        try:
            _buf = s.recv(l)
            buf = _buf
        except socket.error as e:
            if buf is not None:
                break
    return buf



def turnTinker():
    # turn
    client = actionlib.SimpleActionClient('simple_move', SimpleMoveAction)
    client.wait_for_server()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('laptop.furoc.net', 9009))
    s.setblocking(0)
    while not rospy.is_shutdown():
        buf = read_latest(s, 4)
        angle = unpack('f', buf)[0]
        if fabs(angle) > threshold:
            angle *= 0.8
            goal = SimpleMoveGoal()
            goal.target.x = 0
            goal.target.y = 0
            goal.target.theta = angle
            client.send_goal(goal)
            client.wait_for_result()
            read_latest(s, 4)


def main():
    rospy.init_node('listen_and_move', anonymous=False)
    rospy.loginfo('starting listen and move...')
    turnTinker()


if __name__=='__main__':
    main()

