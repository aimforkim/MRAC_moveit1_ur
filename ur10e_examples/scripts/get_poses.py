#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from move_group_utils.move_group_utils import MoveGroupUtils
import keyboard

while True:
    print("press 's' to save pose")
    if keyboard.is_pressed('s'):
        print('saving pose')
    elif keyboard.is_pressed('q'):
        print('quit')
        break
