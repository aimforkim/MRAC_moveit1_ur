#!/usr/bin/env python3

from math import pi

import rospy
from moveit_commander.conversions import list_to_pose
from move_group_utils.move_group_utils import (
    MoveGroupUtils, publish_trajectory_markers, poses_list_from_yaml)
from typing import List
from geometry_msgs.msg import Point, Pose, Quaternion
from pilz_robot_program.pilz_robot_program import Lin, Ptp, Sequence


def pose_from_list(pose_list: List[float]) -> Pose:
    pose = Pose(
        position=Point(pose_list[0], pose_list[1], pose_list[2]),
        orientation=Quaternion(pose_list[3], pose_list[4], pose_list[5], pose_list[6]))

    return pose


def robot_program():

    mgi = MoveGroupUtils()
    # rospy.sleep(1.0)
    home = (0.0, -pi / 2.0, pi / 2.0, -pi, -pi/2.0, 0.0)

    sequence = Sequence()
    sequence.append(Ptp(goal=home, vel_scale=0.2, acc_scale=0.2))

    # read poses from gh_rosparam
    pose_list = rospy.get_param('gh_poses')
    pose_goals = [pose_from_list(pose)for pose in pose_list]

    # # visualize
    # mgi.publish_pose_array(pose_goals)

    p1 = pose_goals[0]
    sequence.append(Lin(goal=p1, vel_scale=0.2, acc_scale=0.2))

    for p in pose_goals[1:]:
        sequence.append(Ptp(goal=p, vel_scale=0.2, acc_scale=0.2))

    sequence.append(Ptp(goal=home, vel_scale=0.2, acc_scale=0.2))

    success, plan = mgi.sequencer.plan(sequence)[:2]

    if not success:
        return rospy.logerr(f'{mgi.name}: Failed to plan to sequence')
    mgi.sequencer.execute()

    return rospy.loginfo('Robot program completed')


if __name__ == '__main__':

    robot_program()
