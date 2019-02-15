#!/usr/bin/env python
import warnings

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Wrench, Point, Vector3

from lwr_controllers.msg import CartesianImpedancePoint

from boris_tools.controller_manager_interface import ControllerManagerInterface


class CartesianImpedanceCommander(object):


    def __init__(self, ns='left_arm'):

        assert ns in ["left_arm", "right_arm"]

        self._base_link = '%s_base_link'%(ns,)
        self._tip_link = '%s_7_link'%(ns,)

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self._cartesian_command_pub = rospy.Publisher('%s/cartesian_impedance_controller/command'%(ns,), CartesianImpedancePoint, tcp_nodelay=False, queue_size=1)

        self._cmi = ControllerManagerInterface(ns="%s/"%(ns,))

        if not self._cmi.is_loaded('cartesian_impedance_controller'):
            self._cmi.load_controller('cartesian_impedance_controller')


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

        if not self._cmi.is_running('cartesian_impedance_controller'):
            self._cmi.start_controller('cartesian_impedance_controller')
            self.wait_running('cartesian_impedance_controller')

            self._is_active = True
    

    def stop(self):

        if self._cmi.is_running('cartesian_impedance_controller'):
            self._cmi.stop_controller('cartesian_impedance_controller')
            self.wait_stopped('cartesian_impedance_controller')

            self._is_active = False

        if not self._cmi.is_running('joint_trajectory_controller'):
            self._cmi.start_controller('joint_trajectory_controller')
            self.wait_running('joint_trajectory_controller')

    def is_active(self):

        return self._is_active
    # def stop_special(self):


    #     if self._cmi.is_running('cartesian_impedance_controller'):
    #         print "Stopping Cartesian Impedance controller!"
    #         self._cmi.stop_controller('cartesian_impedance_controller')
    #     rospy.sleep(3.0)

        

    #     # rospy.sleep(1.5)
    #     if not self._cmi.is_running('joint_impedance_controller'):
    #         print "Starting Impedance controller!"
    #         self._cmi.start_controller('joint_impedance_controller')
    #         rospy.sleep(2.0)
    #         print "Stopping Impedance controller!"
    #         self._cmi.stop_controller('joint_impedance_controller')
    #         rospy.sleep(2.0)


    #     if not self._cmi.is_running('joint_trajectory_controller'):
    #         self._cmi.start_controller('joint_trajectory_controller')
    #         print "Starting Position controller!"
    #         rospy.sleep(5.0)
        
    #     rospy.sleep(1.5)
        

        # rospy.sleep(1.5)


    def compute_command(self, ee_goal):

        return self.compute_command_cart_imp(ee_goal)

    def compute_command_cart_imp(self, 
                        ee_goal, # (position, quaternion)
                        impedance=(300,300,300,30,30,30), 
                        damping=(0.5,0.5,0.5,0.5,0.5,0.5),
                        force=(0,0,0),
                        torque=(0,0,0)):

        cmd = CartesianImpedancePoint()

        cmd.x_FRI = Pose(position=Point(*ee_goal[0]), orientation=Quaternion(*ee_goal[1]))

        cmd.k_FRI.x = impedance[0]
        cmd.k_FRI.y = impedance[1]
        cmd.k_FRI.z = impedance[2]
        cmd.k_FRI.rx = impedance[3]
        cmd.k_FRI.ry = impedance[4]
        cmd.k_FRI.rz = impedance[5]

        cmd.d_FRI.x = damping[0]
        cmd.d_FRI.y = damping[1]
        cmd.d_FRI.z = damping[2]
        cmd.d_FRI.rx = damping[3]
        cmd.d_FRI.ry = damping[4]
        cmd.d_FRI.rz = damping[5]

        cmd.f_FRI = Wrench(force=Vector3(*force), torque=Vector3(*torque))

        cmd.header.frame_id = self._base_link


        return cmd


    def send_command(self, cmd):

        ## TODO: check command before sending for safety reasons

        self._cartesian_command_pub.publish(cmd)

    def get_current_pose(self):

        from boris_tools.ros_tf_utils import TFManager

        tfm = TFManager()

        p, q, t = tfm.get_transform(self._base_link,self._tip_link)

        pose = (p,q)


        return pose


if __name__ == '__main__':
    rospy.init_node("test_tf_manager")

    cic = CartesianImpedanceCommander()

    pose = cic.get_current_pose()

    cic.activate()

    cmd = cic.compute_command_cart_imp(pose)

    p, q = pose
    
    p2 = (p[0]+0.03,p[1],p[2])

    pose2 = (p2,q)

    cmd2 = cic.compute_command_cart_imp(pose2)




