

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

from boris_joint_trajectory_action import MinJerkTrajHelper
import numpy as np

def main():

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

    trajectory, _ = parse_trajectory_file('contact_trajectories/'+trajectories[1])

    traj_msg = make_ros_trajectory_msg(trajectory,joint_names, index_map=(1,8))
    traj_hand_msg = make_ros_trajectory_msg(trajectory,hand_joint_names[:1], index_map=(8,9))
    
    cartesian_traj = make_cartesian_trajectory(trajectory, index_map=(1,8), fk_func=kin.forward_kinematics)

    traj_helper = MinJerkTrajHelper()
    traj_helper_hand = MinJerkTrajHelper()
    
    traj_helper_hand.set_waypoints(traj_hand_msg)

    traj_helper.set_waypoints(traj_msg)
    print traj_msg.points[0].time_from_start.to_sec()

    time_steps = np.linspace(0,1,1000)

    boris.exit_control_mode()
    boris.set_control_mode(mode="joint_impedance", limb_name="left_arm")

    boris.goto_with_moveit("left_arm",traj_msg.points[0].positions)

    

    i = 0
    loop_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        
        # key = raw_input("key: ")
        # if key=='n' and i < len(time_steps)-1:
        #     i += 1
        # elif key=='b' and i > 0:
        #     i -= 1
        

        joint_goal = traj_helper.get_point_t(time_steps[i])

        hand_goal = traj_helper_hand.get_point_t(time_steps[i])
        print "Goal: ", hand_goal.time_from_start.to_sec(), " i=", i, " pos: ",hand_goal.positions

        cmd = boris.cmd_joint_angles(angles=joint_goal.positions,execute=True)
        boris.goto_joint_angles('left_hand',hand_goal.positions)

        loop_rate.sleep()

        i += 1
        if i >= len(time_steps)-1:
            i = len(time_steps)-1
    # for i in np.linspace(0,1,10):
    #     print traj_helper.get_point_t(i).time_from_start.to_sec()
    
    
    
    # boris.exit_control_mode()
    

    # 
    # plan_traj = plan.joint_trajectory
    
    # t0 = rospy.Time.now()
    # i = 0
    # n_wpts = len(plan_traj.points)
    # total_time = plan_traj.points[-1].time_from_start.to_sec()
    # frequency = n_wpts/total_time

    # print "Frequency: ", frequency
    # rate = rospy.Rate(frequency)

    # # boris.set_control_mode(mode="joint_impedance", limb_name="left_arm")

    # while not rospy.is_shutdown():

    #     pass

    # cmd = None
    # for joint_goal in (plan_traj.points):
        
    #     cmd = boris.cmd_joint_angles(angles=joint_goal.positions,execute=True)

       
        
    #     print "Time: ", (rospy.Time.now()-t0).to_sec(), " Expect: ", joint_goal.time_from_start.to_sec()
    #     print "Cmd: ", cmd.data
    #     i+=1
    #     # c = raw_input("next: ")
        
    #     if rospy.is_shutdown():
    #         break

    #     rate.sleep()

    # tf = rospy.Time.now()

    # print "Final Time: ", (tf-t0).to_sec(), " Expect: ", trajectory[-1][0]



    rospy.sleep(3.0)

if __name__ == '__main__':
    main()
