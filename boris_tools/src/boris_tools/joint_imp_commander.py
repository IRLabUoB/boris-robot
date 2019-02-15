#!/usr/bin/env python
import warnings

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

from lwr_controllers.msg import CartesianImpedancePoint

from boris_tools.controller_manager_interface import ControllerManagerInterface


class JointImpedanceCommander(object):


    def __init__(self, ns='left_arm'):

        assert ns in ["left_arm", "right_arm"]

        self._ns = ns

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")

            self._command_pub = rospy.Publisher('%s/joint_impedance_controller/command'%(self._ns,), Float64MultiArray, tcp_nodelay=False, queue_size=None)
            self._command_stiffness_pub = rospy.Publisher('%s/joint_impedance_controller/stiffness'%(self._ns,), Float64MultiArray, tcp_nodelay=False, queue_size=None)
            self._command_damping_pub = rospy.Publisher('%s/joint_impedance_controller/damping'%(self._ns,), Float64MultiArray, tcp_nodelay=False, queue_size=None)

        self._cmi = ControllerManagerInterface(ns="%s/"%(ns,))

        if not self._cmi.is_loaded('joint_impedance_controller'):
            self._cmi.load_controller('joint_impedance_controller')


        self._is_active = False

        
            
    def wait_loaded(self, name):

        while not self._cmi.is_loaded(name) and not rospy.is_shutdown():
            rospy.sleep(0.25)

    def wait_running(self, name):

        while not self._cmi.is_running(name) and not rospy.is_shutdown():
            rospy.sleep(0.25)

    def wait_stopped(self, name):

        while self._cmi.is_running(name) and not rospy.is_shutdown():
            rospy.sleep(0.25)

    def activate(self):

        if self._cmi.is_running('joint_trajectory_controller'):
            self._cmi.stop_controller('joint_trajectory_controller')

            self.wait_stopped('joint_trajectory_controller')

        if not self._cmi.is_running('joint_impedance_controller'):
            self._cmi.start_controller('joint_impedance_controller')
            self.wait_running('joint_impedance_controller')

        self._is_active = self._cmi.is_running('joint_impedance_controller')
    

    def stop(self):

        if self._cmi.is_running('joint_impedance_controller'):
            self._cmi.stop_controller('joint_impedance_controller')
            self.wait_stopped('joint_impedance_controller')

        self._is_active = self._cmi.is_running('joint_impedance_controller')

        if not self._cmi.is_running('joint_trajectory_controller'):
            self._cmi.start_controller('joint_trajectory_controller')
            self.wait_running('joint_trajectory_controller')

    def is_active(self):

        return self._is_active

    def compute_command(self, joint_goal):
        cmd = Float64MultiArray()
        cmd.data = joint_goal
        return cmd


    def send_command(self, cmd):

        ## TODO: check command before sending for safety reasons

        self._command_pub.publish(cmd)

    def send_damping(self, damping):
        cmd = Float64MultiArray()
        cmd.data = damping
        self._command_damping_pub.publish(cmd)
    
    def send_stiffness(self, stiffness):
        cmd = Float64MultiArray()
        cmd.data = stiffness
        self._command_stiffness_pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node("test_joint_imp_commander")

    jic = JointImpedanceCommander()





