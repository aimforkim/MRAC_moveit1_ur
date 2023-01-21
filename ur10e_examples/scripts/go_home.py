#!/usr/bin/env python3

from math import pi
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

from move_group_utils.move_group_utils import (
    MoveGroupUtils, publish_trajectory_markers)
from pilz_robot_program.pilz_robot_program import (Circ, Lin, Ptp, Sequence)


def robot_program():

    mgi = MoveGroupUtils()
    rospy.sleep(1.0)

    # home = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, -pi / 2.0)
    home = (0.0, -pi / 2.0, pi / 2.0, -pi, -pi/2.0, 0.0)

    success, plan = mgi.sequencer.plan(
        Ptp(goal=home, vel_scale=0.2, acc_scale=0.2))[:2]

    if not success:
        return rospy.logerr('Failed to plan to home position')
    mgi.sequencer.execute()

    # initialize sequence
    sequence = Sequence()

    # append commands to sequence
    sequence.append(Ptp(goal=home))

    if not success:
        return rospy.logerr('Failed to plan sequence')
    mgi.sequencer.execute()

    return rospy.loginfo('Robot program completed')


if __name__ == '__main__':

    robot_program()
