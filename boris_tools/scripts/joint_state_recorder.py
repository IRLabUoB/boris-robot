#!/usr/bin/env python

import argparse
import rospy

from sensor_msgs.msg import JointState
from getch import getch

import numpy as np


waypoints = []


def joint_state_callback(joint_state_msg):

    c = getch()

    if c in ['\x1b', '\x03']:
        rospy.signal_shutdown("Acquisition finished.")

    elif c in ['a']:

        joint_angles = np.array(joint_state_msg.position)
        waypoints.append(joint_angles)
        print("Added new waypoint. Number of waypoints %d."%(len(waypoints,)))
        print("Joint angles: ", joint_angles)
        
    elif c in ['d']:

        if len(waypoints) > 0:
            del waypoints[-1]
            print("Deleted last waypoint. Number of waypoints %d."%(len(waypoints,)))
        else:
            print("Waypoint list is empty.")
    elif c in ['s']:

        print("Saving current waypoint list.")
        filename = raw_input("Filename:")
        np.save('%s.npy'%(filename,), waypoints)
        print("Saved and quit")
        rospy.signal_shutdown("Acquisition finished.")




def main():
    
    print("Initiawaypoints[-1]izing node... ")
    rospy.init_node("joint_state_recorder_node")
    print("Getting robot state... ")

    joint_state_sub  = rospy.Subscriber("/right_arm/joint_states", JointState, joint_state_callback)

    rate = rospy.Rate(10)

    print("Press A on the keyboard to acquire a joint pose")
    print("Press ESC on the keyboard to quit")
    while not rospy.is_shutdown():

        rate.sleep()



if __name__ == '__main__':
    main()