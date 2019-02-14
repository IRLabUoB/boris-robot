

#!/usr/bin/env python

import sys
from copy import copy
import rospy
from getch import getch
import numpy as np

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



    rospy.init_node('test_cartesian_trajectory')

    kin  = boris_kinematics(root_link="left_arm_base_link")
    boris = BorisRobot()

    jic = JointImpedanceCommander()

    jic.stop()

    rate = rospy.Rate(30)

    jic.activate()

    joint_names = boris.joint_names()
    print joint_names[:7]

    neg_keys = ['a','s','d','f','g','h','j']
    pos_keys = ['q','w','e','r','t','y','u']
    key_map_neg = dict(zip(neg_keys,range(7)))
    key_map_pos = dict(zip(pos_keys,range(7)))

    delta_angle = 0.05
    jic.send_damping([25,25,25,25,25,5,2.0])#[0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    jic.send_stiffness([200,200,200,100,100,50,25]) #[800,800,800,800,300,300,500]

    while not rospy.is_shutdown():
        joint_angles = boris.joint_angles()
        angles = np.array([joint_angles[name] for name in joint_names[:7]])
        current_angles = np.array([joint_angles[name] for name in joint_names[:7]])

        key = getch(timeout=0.5)#raw_input("key: ")

        if key in pos_keys:

            idx = key_map_pos[key]

            angles[idx] += delta_angle
            
            cmd = jic.compute_command(angles)

            print "Current: ", current_angles
            print "Command: ", cmd.data
            jic.send_command(cmd)
        elif key in neg_keys:

            idx = key_map_neg[key]

            angles[idx] -= delta_angle
            
            cmd = jic.compute_command(angles)

            print "Current: ", current_angles
            print "Command: ", cmd.data
            jic.send_command(cmd)
        elif key in ['\x1b', '\x03']:
                rospy.signal_shutdown("Finished.")
                break





        rate.sleep()


    
if __name__ == '__main__':
    main()
