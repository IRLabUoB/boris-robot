#!/usr/bin/env python


import argparse, sys
from copy import copy

import rospy


from recorder import JointRecorder
from jsk_gui_msgs.msg import TouchEvent
import numpy as np
from trajectory_action_client import Trajectory





traj = None
recording = False
pressed = False
t = 0.0


def on_touch(msg):

    global recording, pressed, traj, t

    if msg.state==2:

        if not pressed:
            
            pressed = True
            hand_goal = (msg.y/msg.h)
            print("Is touching!! %lf",hand_goal)

            # t = 0.0
            # traj.add_point([0.0], t)
            t += 0.5    
            traj.add_point([hand_goal], t)
            traj.start()
            traj.wait()
            # traj.stop()
            traj.clear()
    elif msg.state==1:

        if pressed:
            pressed = False





def main():
    global traj

    print("Initialising node... ")
    rospy.init_node("joint_trajectory_recorder_node")
    traj = Trajectory()
    joint_state_sub  = rospy.Subscriber("/touchEvent", TouchEvent, on_touch, queue_size=1, tcp_nodelay=True)

    rate = rospy.Rate(60)
    
    while not rospy.is_shutdown():
        # print("hand angle: %lf"%(robot.joint_angle('left_hand_synergy_joint')))
        
        rate.sleep()



if __name__ == '__main__':
    main()