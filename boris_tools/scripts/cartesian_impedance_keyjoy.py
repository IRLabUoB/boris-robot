#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Wrench, Point, Vector3
import tf
from tf import TransformListener

from lwr_controllers.msg import CartesianImpedancePoint

from boris_tools.cartesian_imp_commander import CartesianImpedanceCommander

import sys

NODE_NAME = 'boris_cartesian_imp_traj_player'   


class CartesianTrajectoryFollower(object):
    def __init__(self):
    
    
        self._arm_base_frame = rospy.get_param('%s/base_frame'%(NODE_NAME,),'left_arm_base_link')
        self._world_base_frame = rospy.get_param('%s/robot_base_frame'%(NODE_NAME,),'world')
        self._ee_frame = rospy.get_param('%s/ee_frame'%(NODE_NAME,),'left_arm_7_link')

        self._save_calib_file = rospy.get_param('%s/save_calib_file'%(NODE_NAME,),'hand_eye_calib_data.npy')
        self._load_calib_file = rospy.get_param('%s/load_calib_file'%(NODE_NAME,),'hand_eye_calib_data.npy')
        self._calib_from_file = rospy.get_param('%s/calibrate_from_file'%(NODE_NAME,),False)

        rospy.loginfo("World frame: %s"%(self._world_base_frame,))
        rospy.loginfo("Robot frame: %s"%(self._arm_base_frame,))
        rospy.loginfo("End-effector frame: %s"%(self._ee_frame,))


        self._cic = CartesianImpedanceCommander()

        self._cic.activate()

    def makeCartesianImpCmd(self, key, init_cip = None):
        

        cip = CartesianImpedancePoint()
        cip.x_FRI = Pose(position=Point(*[0,0,0]), orientation=Quaternion(*[0,0,0,1]))

        if init_cip is not None:
            cip = init_cip
            print "Set the stuff"

        incr = 0.05
        r_incr = 0.1
        
        if key=="w":
            cip.x_FRI.position.x += incr
        if key=="s":
            cip.x_FRI.position.x -= incr
        if key=="d":
            cip.x_FRI.position.y += incr
        if key=="a":
            cip.x_FRI.position.y -= incr
        if key=="q":
            cip.x_FRI.position.z -= incr
        if key=="e":
            cip.x_FRI.position.z += incr
        
        
        omega = list(tf.transformations.euler_from_quaternion([cip.x_FRI.orientation.x, cip.x_FRI.orientation.y, cip.x_FRI.orientation.z, cip.x_FRI.orientation.w]))
        if key=="u":
            omega[0] += r_incr
        if key=="j":
            omega[0] -= r_incr
        if key=="h":
            omega[1] += r_incr
        if key=="k":
            omega[1] -= r_incr
        if key=="y":
            omega[2] -= r_incr
        if key=="i":
            omega[2] += r_incr


        q_new = tf.transformations.quaternion_from_euler(*omega)
        cip.x_FRI.orientation = Quaternion(*q_new)
        # print "Look: ", cip.x_FRI.position, cip.x_FRI.orientation
        cip.f_FRI = Wrench(force=Vector3(*np.zeros(3)), torque=Vector3(*np.zeros(3)))
        cip.header.frame_id = "left_arm_base_link"
        return cip

            

    def run(self):
        from getch import getch
        finish = False
        print("Press enter to record to next. Add/Remove/Calibrate/Help? (a/r/c/h)")
        rate = rospy.Rate(500)

        while not finish:
            
            r = getch()

            if r == 'l':
                ee_t, ee_q = self._cic.get_current_pose()
                print "------------%Pose---------------"
                print "End-effector transform: ", ee_t, ee_q
                print "---------------------------------"

            
            elif r in ['\x1b', '\x03']:
                rospy.signal_shutdown("Finished.")
                break
            elif r is not None:
                
    
                pose = self._cic.get_current_pose()
                cmd = self._cic.compute_command_cart_imp(pose)
                # cmd.k_FRI.x = cmd.k_FRI.y = cmd.k_FRI.z = 5000
                # cmd.k_FRI.rx = cmd.k_FRI.ry = cmd.k_FRI.rz = 2000

                if r is not None:
                    cmd = self.makeCartesianImpCmd(r,init_cip=cmd)
                self._cic.send_command(cmd)
                print cmd.k_FRI

        

    
            rate.sleep()


def main():
    ctf = CartesianTrajectoryFollower()

    ctf.run()


    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)

    main()
