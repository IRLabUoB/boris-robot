

import sys
from copy import copy
import rospy


from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)

from boris_tools.moveit_wrapper import MoveitWrapper
from boris_tools.trajectory_io import parse_trajectory_file, make_ros_trajectory_msg
from boris_tools.boris_robot import BorisRobot

def main():

    moveit_wrapper = MoveitWrapper()

    moveit_wrapper.init_moveit_commander()
    rospy.init_node('aml_trajectory_player')

    moveit_wrapper.setup()

    boris = BorisRobot(moveit_wrapper = moveit_wrapper)

    # trajectory = [-2.02830171585, 1.04696202278, 1.55436050892, -1.44723391533, -0.815707325935, 0.387918055058, -2.68925023079]
    joint_names = rospy.get_param("left_arm/joints")
    hand_joint_names = rospy.get_param("left_hand/joints")

    trajectories = ['suitcase_trajectory01.csv',
                    'cylinder_traj.csv', 
                    'cylinder_slide_trajectory.csv', 
                    'test_traj.csv', 
                    'box_traj.csv',
                    'ibuprofen_trajectory.csv']

    trajectory, _ = parse_trajectory_file(trajectories[5])

    traj_msg = make_ros_trajectory_msg(trajectory,joint_names, index_map=(1,8))
    traj_hand_msg = make_ros_trajectory_msg(trajectory,hand_joint_names[:1], index_map=(8,9))



    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        c = raw_input("send: ")
        if c == "y":
            boris.follow_trajectory("left_arm",traj_msg,first_waypoint_moveit=True)
            boris.follow_trajectory("left_hand",traj_hand_msg,first_waypoint_moveit=False)  

        if c == "h":
            boris.stop_trajectories()  

        rate.sleep()


if __name__ == '__main__':
    main()
