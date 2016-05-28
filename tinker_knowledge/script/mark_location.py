#!/usr/bin/python
import os
import rospkg

import rospy
from geometry_msgs.msg import PoseStamped
from tinker_knowledge.position import Pose, PoseList

from termcolor import colored


def mark_pose(pose, pose_list):
    p = Pose.R(pose.pose)
    name = raw_input('Input location name:')
    p.name = name
    pose_list.pose.append(p)


def main():
    rospack = rospkg.RosPack()
    output_path = os.path.join(rospack.get_path('tinker_knowledge'), 'position')
    if not os.path.exists(output_path):
        os.makedirs(output_path)
        rospy.loginfo(colored('create path %s', 'yellow'), output_path)

    rospy.init_node('mark_location', anonymous=False)
    rospy.loginfo(colored('starting mark location on map ...', 'green'))
    rospy.loginfo(colored('Please publish target pose in rviz', 'green'))
    list_name = raw_input('Input task name:')
    pose_list = PoseList(name=list_name)
    rospy.Subscriber('/mark_goal', PoseStamped, callback=mark_pose, callback_args=pose_list)
    rospy.spin()

    output_file = os.path.join(output_path, list_name + '.xml')
    with open(output_file, 'w') as f:
        f.write(pose_list.render(pretty=True))
        rospy.loginfo(colored('poselist saved %s', 'green'), output_file)


if __name__ == '__main__':
    main()
