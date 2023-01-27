#!/usr/bin/env python3
from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from move_group_utils.move_group_utils import MoveGroupUtils


def robot_program():

    # initialize node and moveit commander
    mgi = MoveGroupUtils()

    poses = []
    pose = mgi.move_group.get_current_pose()
    joint = mgi.move_group.get_current_joint_values()

    print(pose)
    print(joint)
    # while True:
    #     print("press 's' to save pose")
    #     if keyboard.is_pressed('s'):
    #         print('"saving pose"')
    #         pose = mgi.move_group.get_current_pose()
    #         poses.append(pose)
    #     elif keyboard.is_pressed('q'):
    #         print('quitting')
    #         break


if __name__ == '__main__':

    robot_program()
