#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Wrench, Point, Vector3
import tf
from tf import TransformListener

from boris_tools.boris_robot import BorisRobot

import sys, atexit

NODE_NAME = 'boris_gravity_compesantion'   


class GravityCompensation(object):
    def __init__(self):
    

        self._robot = BorisRobot()

        # atexit.register(self._cic.stop)


    def run(self):
        from getch import getch
        finish = False
        print("Press enter to record to next. Add/Remove/Calibrate/Help? (a/r/c/h)")
        rate = rospy.Rate(10)

       
        self._robot.wait_enabled()

        self._robot.set_gravity_compesantion_mode()
        while not finish:
            

            print self._robot.angles('left_arm')

            rate.sleep()
            finish = rospy.is_shutdown()


def main():
    ctf = GravityCompensation()

    ctf.run()


    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)

    main()
