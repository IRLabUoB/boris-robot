#!/usr/bin/env python


import argparse, sys
from copy import copy

import rospy


from recorder import JointRecorder
import numpy as np
from boris_robot import BorisRobot





recording = False
pressed = False
robot = BorisRobot()


def main():

    print("Initialising node... ")
    rospy.init_node("joint_trajectory_recorder_node")

    rate = rospy.Rate(20)
    
    # print traj.result()
    recorder = JointRecorder(sys.argv[1],30.0, robot)

    rospy.on_shutdown(recorder.stop)
    print("Recording. Press Ctrl-C to stop.")
    recorder.record()
    # while not rospy.is_shutdown():
    #     # print("hand angle: %lf"%(robot.joint_angle('left_hand_synergy_joint')))
        
    #     rate.sleep()



if __name__ == '__main__':
    main()
