#!/usr/bin/env python3
from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from move_group_utils.move_group_utils import MoveGroupUtils
<<<<<<< HEAD


def robot_program():

    # initialize node and moveit commander
    mgi = MoveGroupUtils()

    poses = []
    pose = mgi.move_group.get_current_pose()

    print(pose)

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
=======
import keyboard

# def getKey():
#     settings = termios.tcgetattr(sys.stdin)

#     tty.setraw(sys.stdin.fileno())
#     rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
#     if rlist:
#         key = sys.stdin.read(1)
#     else:
#         key = ''

#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key
while True:
    print("press 's' to save pose")
    if keyboard.is_pressed('s'): 
        print('"saving pose"')
        # pose = mgi.move_group.get_current_pose()
        # poses.append(pose)
    elif keyboard.is_pressed('q'):
        print('quitting')
        break

# mgi = MoveGroupUtils()

# poses = []
# pose = mgi.move_group.get_current_pose()
# while True:
#     print("press 's' to save pose")
#     if keyboard.is_pressed('s'): 
#         print('"saving pose"')
#         pose = mgi.move_group.get_current_pose()
#         poses.append(pose)
#     elif keyboard.is_pressed('q'):
#         print('quitting')
#         break


# def robot_program():

#     # initialize node and moveit commander
#     mgi = MoveGroupUtils()

#     poses = []
#     pose = mgi.move_group.get_current_pose()
#     while True:
#         print("press 's' to save pose")
#         if keyboard.is_pressed('s'): 
#             print('"saving pose"')
#             pose = mgi.move_group.get_current_pose()
#             poses.append(pose)
#         elif keyboard.is_pressed('q'):
#             print('quitting')
#             break
            


>>>>>>> 2205e931378c72b857012273dc1b2d58e16cf2c5
