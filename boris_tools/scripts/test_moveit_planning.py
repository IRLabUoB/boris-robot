

#!/usr/bin/env python

import sys
from copy import copy
import rospy


from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)

from boris_tools.moveit_wrapper import MoveitWrapper
from boris_tools.trajectory_io import parse_trajectory_file, make_ros_trajectory_msg, make_cartesian_trajectory
from boris_tools.boris_robot import BorisRobot
from boris_tools.boris_kinematics import boris_kinematics

from boris_tools.cartesian_imp_commander import CartesianImpedanceCommander

if __name__ == '__main__':
    
    moveit_wrapper = MoveitWrapper()
    moveit_wrapper.init_moveit_commander()

    rospy.init_node('test_cartesian_trajectory')

    moveit_wrapper.setup()
    kin  = boris_kinematics(root_link="left_arm_base_link")
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

    trajectory, _ = parse_trajectory_file('contact_trajectories/'+trajectories[5])

    traj_msg = make_ros_trajectory_msg(trajectory,joint_names, index_map=(1,8))
    traj_hand_msg = make_ros_trajectory_msg(trajectory,hand_joint_names[:1], index_map=(8,9))
    
    cartesian_traj = make_cartesian_trajectory(trajectory, index_map=(1,8), fk_func=kin.forward_kinematics)


    # cic = CartesianImpedanceCommander()
    # cic.stop()

    # Get plan to last trajectory position
    plan = boris.get_moveit_plan('left_arm', traj_msg.points[-1].positions)
    # get ROS trajectory msg: plan.joint_trajectory
    #type(plan.joint_trajectory.points[0])
