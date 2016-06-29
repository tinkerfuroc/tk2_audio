#!/usr/bin/python

import rospy
import roslib
import actionlib
import sys
from tinker_msgs.msg._SimpleMoveGoal import SimpleMoveGoal
from tinker_msgs.msg._SimpleMoveAction import SimpleMoveAction
from std_msgs.msg import Float64

threshold = 0.08726646  # 5 degree


def turnTinker(angle):
    # turn
    client = actionlib.SimpleActionClient('simple_move', SimpleMoveAction)
    client.wait_for_server()
    goal = SimpleMoveGoal()
    goal.target.x = 0
    goal.target.y = 0
    goal.target.theta = angle
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo(str(client.get_result()))

def callback(msg):
    angle = msg.data
    if angle<threshold and angle>-threshold:
        rospy.loginfo('The angle meets the threshold now.')
    elif angle>threshold:
        rospy.loginfo('Tinker should turn left')
        turnTinker(angle)
    elif angle<-threshold:
        rospy.loginfo('Tinker should turn right')    
        turnTinker(angle)
    rospy.sleep(1);

def main():
    rospy.init_node('listen_and_move', anonymous=False)
    rospy.loginfo('starting listen and move...')
    rospy.Subscriber('audio', Float64, callback)
    rospy.spin()

if __name__=='__main__':
    main()

