#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from getch import getch
import numpy as np
import sys

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("joint_state_player_node")


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    arm = moveit_commander.MoveGroupCommander("left_arm")

    waypoints = np.load('calib_sim3.npy')

    rate = rospy.Rate(100)
    joint_angle_target = np.zeros(7)
    selected_idx = -1
    while not rospy.is_shutdown():

        c = getch()

        if c in ['n']:
            selected_idx = (selected_idx+1)%len(waypoints)

            joint_angle_target = waypoints[selected_idx]
            print("Selected joint goal %d: "%(selected_idx,), joint_angle_target)

        elif c in ['b']:
            selected_idx -= 1
            if selected_idx < 0:
                selected_idx = len(waypoints)-1

            joint_angle_target = waypoints[selected_idx]
            print("Selected joint goal %d: "%(selected_idx,), joint_angle_target)

        elif c in ['a']:
            current_joint_angles = arm.get_current_joint_values()
            joint_angles = np.asarray(current_joint_angles)

            waypoints = np.vstack([waypoints,joint_angles])
            print("Added new waypoint. Number of waypoints %d."%(len(waypoints,)))
            print("Joint angles: ", joint_angles)
        
        elif c in ['d']:

            if len(waypoints) > 0:
                waypoints = np.delete(waypoints, (selected_idx), axis=0)
                print("Deleted %dth waypoint. Number of waypoints %d."%(selected_idx+1,len(waypoints,)))
            else:
                print("Waypoint list is empty.")

        elif c in ['e']:

            current_joint_angles = np.asarray(arm.get_current_joint_values())
            
            before = np.array(waypoints[selected_idx])
            waypoints[selected_idx] = current_joint_angles
            print("Editting joint angles %d: "%(selected_idx,), before)
            print("Set to", current_joint_angles)

        elif c in ['s']:
            print("Saving current waypoint list.")
            filename = raw_input("Filename:")
            np.save('%s.npy'%(filename,), waypoints)
            print("Saved.")

        elif c in ['g']:

            try:
                arm.set_joint_value_target(joint_angle_target)
                arm.set_max_velocity_scaling_factor(0.35)
                arm.set_max_acceleration_scaling_factor(0.4)
                arm.go()
            except:
                print("Not possible to go to waypoint. Try another one")
        elif c in ['\x1b', '\x03']:
            rospy.signal_shutdown("Bye.")


        rate.sleep()

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException, e:
        print(e)
