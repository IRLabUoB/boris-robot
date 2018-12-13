#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import tf2_ros
import geometry_msgs.msg
import moveit_msgs.msg
from sensor_msgs.msg import JointState
import tf.transformations as tt
import math
import numpy as np



def open_hand(arm_and_hand):
    print "Open hand"
    currentJointValues = arm_and_hand.get_current_joint_values()
    hand_closed = currentJointValues[:]
    hand_closed[0] = 0.0
    arm_and_hand.set_joint_value_target(hand_closed)
    arm_and_hand.go(wait=True)



def close_hand(arm_and_hand):
    print "Close hand"
    currentJointValues = arm_and_hand.get_current_joint_values()
    hand_closed = currentJointValues[:]
    hand_closed[0] = 1.0
    arm_and_hand.set_joint_value_target(hand_closed)
    arm_and_hand.go(wait=True)



def move(arm_and_hand, ee_target):
    print ("Move to pose")
    arm_and_hand.set_pose_target(ee_target)
    arm_and_hand.go(wait=True)
    arm_and_hand.stop()
    arm_and_hand.clear_pose_targets()



def pickup(arm_group, hand_group, obj_x, obj_y):

    print "\nPicking up object"

    # open hand
    open_hand(hand_group)

    # put the hand into the correct orientation
    # orientation = tt.quaternion_from_euler(1, 1, 0) # (roll, pitch, yaw)
    ee_target = geometry_msgs.msg.Pose()
    ee_target.orientation.x = -0.049
    ee_target.orientation.y = 0.669
    ee_target.orientation.z = 0.045
    ee_target.orientation.w = 0.740
    # [-0.049, 0.669, 0.045, 0.740]
    # define position target
    # [0.612, 0.621, -0.160]
    position = [obj_x, obj_y, 0.05]
    ee_target.position.x = position[0]
    ee_target.position.y = position[1]
    ee_target.position.z = position[2]

    # move arm to the starting position
    move(arm_group, ee_target)

    # move arm to the green box
    ee_target.position.z = -0.16
    move(arm_group, ee_target)

    # close hand
    close_hand(hand_group)

    # move arm up
    ee_target.position.z = 0.05
    move(arm_group, ee_target)



def place(arm_group, hand_group, height):

    print ("\nPlacing object")

    # put the hand into the correct orientation
    orientation = tt.quaternion_from_euler(1, 1, 0) # (roll, pitch, yaw)
    ee_target = geometry_msgs.msg.Pose()
    ee_target.orientation.x = orientation[0]
    ee_target.orientation.y = orientation[1]
    ee_target.orientation.z = orientation[2]
    ee_target.orientation.w = orientation[3]
    
    # define position target
    position = [0.612, 0.1, height+0.05]
    ee_target.position.x = position[0]
    ee_target.position.y = position[1]
    ee_target.position.z = position[2]

    # move arm to the tower
    move(arm_group, ee_target)

    # move arm down
    ee_target.position.z = height
    move(arm_group, ee_target)

    # open hand
    open_hand(hand_group)

    # move arm up
    ee_target.position.z = height+0.05
    move(arm_group, ee_target)



def main():

    # start moveit
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_commander")
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # set up planning group
    arm_group = moveit_commander.MoveGroupCommander("left_hand_arm")
    hand_group = moveit_commander.MoveGroupCommander("left_hand")
    arm_group.set_max_velocity_scaling_factor(0.25)
    arm_group.set_max_acceleration_scaling_factor(0.25)

    # pickup object
    pickup(arm_group, hand_group, 0.612, 0.621)

    # place object
    place(arm_group, hand_group, -0.15)

    # stop moveit
    moveit_commander.roscpp_shutdown()
    return



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException, e:
        print(e)
