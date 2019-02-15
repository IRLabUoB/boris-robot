

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

from boris_tools.joint_imp_commander import JointImpedanceCommander

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


    boris.exit_control_mode()
    

    plan = boris.get_moveit_plan("left_arm",traj_msg.points[0].positions)
    plan_traj = plan.joint_trajectory
    
    t0 = rospy.Time.now()
    i = 0
    n_wpts = len(plan_traj.points)
    total_time = plan_traj.points[-1].time_from_start.to_sec()
    frequency = n_wpts/total_time

    print "Frequency: ", frequency
    rate = rospy.Rate(frequency)

    boris.set_control_mode(mode="joint_impedance", limb_name="left_arm")

    cmd = None
    for joint_goal in (plan_traj.points):
        
        cmd = boris.cmd_joint_angles(angles=joint_goal.positions,execute=True)

       
        
        print "Time: ", (rospy.Time.now()-t0).to_sec(), " Expect: ", joint_goal.time_from_start.to_sec()
        print "Cmd: ", cmd.data
        i+=1
        # c = raw_input("next: ")
        
        if rospy.is_shutdown():
            break

        rate.sleep()

    tf = rospy.Time.now()

    print "Final Time: ", (tf-t0).to_sec(), " Expect: ", trajectory[-1][0]

    t0 = rospy.Time.now()
    i = 0
    n_wpts = len(traj_msg.points)
    total_time = traj_msg.points[-1].time_from_start.to_sec()
    frequency = n_wpts/total_time

    print "Frequency: ", frequency
    rate = rospy.Rate(frequency)
    cmd = None
    boris.follow_trajectory("left_hand",traj_hand_msg,first_waypoint_moveit=False)  
    for joint_goal in (traj_msg.points):
        
        cmd = boris.cmd_joint_angles(angles=joint_goal.positions,execute=True)

       
        
        print "Time: ", (rospy.Time.now()-t0).to_sec(), " Expect: ", joint_goal.time_from_start.to_sec()
        print "Cmd: ", cmd.data
        i+=1
        # c = raw_input("next: ")
        
        if rospy.is_shutdown():
            break

        rate.sleep()

    tf = rospy.Time.now()

    print "Final Time: ", (tf-t0).to_sec(), " Expect: ", trajectory[-1][0]


    rospy.sleep(3.0)

if __name__ == '__main__':
    main()
