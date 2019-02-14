

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


    jic = JointImpedanceCommander()

    jic.stop()

    

    plan = boris.get_moveit_plan("left_arm",traj_msg.points[0].positions)
    plan_traj = plan.joint_trajectory

    # print traj_msg.header.seq
    # print traj_msg.header.stamp
    # print traj_msg.joint_names
    # print traj_msg.points[0].time_from_start.to_sec(), " - ", traj_msg.points[-1].time_from_start.to_sec()
    # print "-----------------"

    # print "Path Size:", len(plan.joint_trajectory.points)
    
    

    # print plan.joint_trajectory.header.seq
    # print plan.joint_trajectory.joint_names
    # print plan.joint_trajectory.header.stamp
    # print plan.joint_trajectory.points[0].time_from_start.to_sec(), " - ", plan.joint_trajectory.points[-1].time_from_start.to_sec()
    # c = raw_input("go:")
    # boris.follow_trajectory("left_arm",plan.joint_trajectory,first_waypoint_moveit=False) 
    # while not rospy.is_shutdown():
    #     c = raw_input("next: ")
         
    #     rospy.sleep(0.1)
    # print kin._base_link
    # print kin._tip_link
    # # print cartesian_traj
    
    t0 = rospy.Time.now()
    i = 0
    n_wpts = len(plan_traj.points)
    total_time = plan_traj.points[-1].time_from_start.to_sec()
    frequency = n_wpts/total_time

    print "Frequency: ", frequency
    rate = rospy.Rate(frequency)

    jic.activate()

    jic.send_damping([25,25,25,25,10,0.01,0.001])#[0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    jic.send_stiffness([200,200,200,100,60,50,10]) #[800,800,800,800,300,300,500]

    cmd = None
    for joint_goal in (plan_traj.points+traj_msg.points):
        
        cmd = jic.compute_command(joint_goal.positions)
        
        print "Time: ", (rospy.Time.now()-t0).to_sec(), " Expect: ", joint_goal.time_from_start.to_sec()
        print "Cmd: ", cmd.data
        i+=1
        c = raw_input("next: ")
        
        if rospy.is_shutdown():
            break


        jic.send_command(cmd)
        rate.sleep()

    tf = rospy.Time.now()

    print "Final Time: ", (tf-t0).to_sec(), " Expect: ", trajectory[-1][0]


    rospy.sleep(3.0)

if __name__ == '__main__':
    main()
