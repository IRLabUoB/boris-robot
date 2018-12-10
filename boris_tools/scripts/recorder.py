#!/usr/bin/env python

import argparse
import rospy

from boris_robot import BorisRobot


class JointRecorder(object):
    def __init__(self, filename, rate, robot):
        """
        Records joint data to a file at a specified rate.
        """

        

        self._filename = filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False

        self._robot = robot
        

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        """
        Stop recording.
        """
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def record(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.
        If a file exists, the function will overwrite existing file.
        """
        if self._filename:
            joint_names = self._robot.joint_names()
            with open(self._filename, 'w') as f:
                f.write('time,')
                f.write(','.join([j for j in joint_names]) + '\n')
                while not self.done():
                    joint_angles = [self._robot.joint_angle(j)
                                    for j in joint_names]
                    f.write("%f," % (self._time_stamp(),))
                    f.write(','.join([str(x) for x in joint_angles]) + '\n')
                    self._rate.sleep()