#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Wrench, Point, Vector3
import tf
from tf import TransformListener

from lwr_controllers.msg import CartesianImpedancePoint

import sys

NODE_NAME = 'boris_eye_calibrator'

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

        self._cartesian_command_pub = rospy.Publisher('left_arm/cartesian_impedance_controller/command', CartesianImpedancePoint, queue_size=0)
    
        self._br = tf.TransformBroadcaster()
        self._tf = TransformListener()

    def broadcast_frame(self, pt, rot, frame_name="marker"):

        rot = np.append(rot, [[0, 0, 0]], 0)
        rot = np.append(rot, [[0], [0], [0], [1]], 1)
        quat = tuple(tf.transformations.quaternion_from_matrix(rot))
        now = rospy.Time.now()
        self.br.sendTransform((pt[0], pt[1], pt[2]), tf.transformations.quaternion_from_matrix(rot), now, frame_name,
                              'base')
        print("should have done it!")


    def get_transform(self, base_frame, source_frame):

        if base_frame == source_frame:
            return (0, 0, 0), (0, 0, 0, 1), rospy.Time.now()

        # t = (0,0,0)
        # q = (0,0,0,1)
        time = None
        while time is None:
            try:

                time = self._tf.getLatestCommonTime(source_frame, base_frame)
            except:
                rospy.loginfo("Failed to get common time between %s and %s. Trying again..."%(source_frame,base_frame,))

        t = None
        q = None
        try:
            t, q = self._tf.lookupTransform(base_frame, source_frame, time)#self._tf_buffer.lookup_transform(frame_name, base_frame, rospy.Time(0), rospy.Duration(5.0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Transform could not be queried.")

            return (0, 0, 0), (0, 0, 0, 1), rospy.Time.now()

        translation = (t[0], t[1], t[2])
        quaternion = (q[0], q[1], q[2], q[3])
        # translation = (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
        # quaternion = (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)

        return translation, quaternion, time

    def to_transform_matrix(self, t, q):

        t_mat = tf.transformations.translation_matrix(t)
        r_mat = tf.transformations.quaternion_matrix(q)
        transform_mat = np.dot(t_mat, r_mat)
        
        return transform_mat

    def to_euler(self, transform):
        return np.asarray(tf.transformations.euler_from_matrix(transform))

    def to_translation(self,transform):

        return np.array(transform[:3,3])


    def makeCartesianImpCmd(self, key, init_cip = None):
        

        cip = CartesianImpedancePoint()
        cip.x_FRI = Pose(position=Point(*[0,0,0]), orientation=Quaternion(*[0,0,0,1]))

        if init_cip is not None:
            cip = init_cip
            print "Set the stuff"

        

        cip.k_FRI.x = 300
        cip.k_FRI.y = 300
        cip.k_FRI.z = 300
        cip.k_FRI.rx = 25
        cip.k_FRI.ry = 25
        cip.k_FRI.rz = 25

        cip.d_FRI.x = 0.5
        cip.d_FRI.y = 0.5
        cip.d_FRI.z = 0.5
        cip.d_FRI.rx = 0.5
        cip.d_FRI.ry = 0.5
        cip.d_FRI.rz = 0.5

        incr = 0.05
        r_incr = 0.05
        
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
        
        

        omega = Quaternion(*[0,0,0,1])
        if key=="u":
            omega.x += r_incr
        if key=="j":
            omega.x -= r_incr
        if key=="h":
            omega.y += r_incr
        if key=="k":
            omega.y -= r_incr
        if key=="y":
            omega.z -= r_incr
        if key=="i":
            omega.z += r_incr


        q_new = tf.transformations.quaternion_multiply([omega.x,omega.y, omega.z, omega.w], 
                                                       [cip.x_FRI.orientation.x, cip.x_FRI.orientation.y, cip.x_FRI.orientation.z, cip.x_FRI.orientation.w])
        cip.x_FRI.orientation = Quaternion(*q_new)
        print "Look: ", cip.x_FRI.position, cip.x_FRI.orientation
        cip.f_FRI = Wrench(force=Vector3(*np.zeros(3)), torque=Vector3(*np.zeros(3)))
        return cip

            

    def run(self):
        from getch import getch
        finish = False
        print("Press enter to record to next. Add/Remove/Calibrate/Help? (a/r/c/h)")
        rate = rospy.Rate(500)

        while not finish:
            
            r = getch()

            if r == 'l':
                ee_t, ee_q, _ = self.get_transform(self._arm_base_frame, self._ee_frame)

                ee_transform = self.to_transform_matrix(ee_t, ee_q)
                print "------------%Pose---------------"
                print "End-effector transform: ", ee_t, ee_q
                print "---------------------------------"
            
                # cip = CartesianImpedancePoint()
    
                # cip.x_FRI = Pose(position=ee_t, orientation=Quaternion(*ee_q))
        
                # cip.k_FRI.x = 300
                # cip.k_FRI.y = 300
                # cip.k_FRI.z = 300
                # cip.k_FRI.rx = 50
                # cip.k_FRI.ry = 50
                # cip.k_FRI.rz = 50

                # cip.d_FRI.x = 0.5
                # cip.d_FRI.y = 0.5
                # cip.d_FRI.z = 0.5
                # cip.d_FRI.rx = 0.5
                # cip.d_FRI.ry = 0.5
                # cip.d_FRI.rz = 0.5

            
            elif r in ['\x1b', '\x03']:
                rospy.signal_shutdown("Finished.")
                break
            elif r is not None:
                
                cip = CartesianImpedancePoint()
                ee_t, ee_q, _ = self.get_transform(self._arm_base_frame, self._ee_frame)

                cip.x_FRI = Pose(position=Point(*ee_t), orientation=Quaternion(*ee_q))
                cmd = self.makeCartesianImpCmd(r,init_cip=cip)

                print cmd.x_FRI.position


                self._cartesian_command_pub.publish(cmd)

        

    
            rate.sleep()





def main():
    ctf = CartesianTrajectoryFollower()

    ctf.run()


    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)

    main()
