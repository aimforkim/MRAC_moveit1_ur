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


if __name__ == "__main__":
    mgi = MoveGroupUtils()
    if rospy.has_param('gh_poses'):
        pose_list = rospy.get_param('gh_poses')
        # print(pose_list)
        pose_goals = [pose_from_list(pose)for pose in pose_list]

        print(pose_goals)

        # visualize the toolpath
        mgi.publish_pose_array(pose_goals)
    else:
        print("no poses")
