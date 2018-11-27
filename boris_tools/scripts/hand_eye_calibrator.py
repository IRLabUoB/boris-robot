#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
import tf
from tf import TransformListener

from aml_calib.hand_eye_calib import HandEyeCalib

import sys

NODE_NAME = 'boris_eye_calibrator'

class HandEyeCalibrator(object):
    def __init__(self):
    
        self._camera_frame = rospy.get_param('%s/camera_frame'%(NODE_NAME,),'camera_rgb_optical_frame')
        self._marker_frame = rospy.get_param('%s/marker_frame'%(NODE_NAME,),'marker26')
        self._robot_base_frame = rospy.get_param('%s/robot_base_frame'%(NODE_NAME,),'world')
        self._ee_frame = rospy.get_param('%s/ee_frame'%(NODE_NAME,),'left_arm_7_link')

        self._save_calib_file = rospy.get_param('%s/save_calib_file'%(NODE_NAME,),'hand_eye_calib_data.npy')
        self._load_calib_file = rospy.get_param('%s/load_calib_file'%(NODE_NAME,),'hand_eye_calib_data.npy')
        self._calib_from_file = rospy.get_param('%s/calibrate_from_file'%(NODE_NAME,),False)

        rospy.loginfo("Camera frame: %s"%(self._camera_frame,))
        rospy.loginfo("Marker frame: %s"%(self._marker_frame,))
        rospy.loginfo("Robot frame: %s"%(self._robot_base_frame,))
        rospy.loginfo("End-effector frame: %s"%(self._ee_frame,))
        rospy.loginfo("Calibration filename to save: %s"%(self._save_calib_file,))
        rospy.loginfo("Calibration filename to load: %s"%(self._load_calib_file,))
        rospy.loginfo("Calibrate from file: %s"%(self._calib_from_file,))

        self._camera_base_p = [0.000, 0.022, 0.0]
        self._camera_base_q = [-0.500, 0.500, -0.500, 0.500]

        self._camera_base_transform = self.to_transform_matrix(self._camera_base_p, self._camera_base_q)

        self._calib = HandEyeCalib()

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


    def calibrate_from_file(self):


        calib_data = np.load(self._load_calib_file).item()

        camera_poses = calib_data['camera_poses']
        ee_poses = calib_data['ee_poses']

        for i in range(len(camera_poses)):
            self._calib.add_measurement(ee_poses[i], camera_poses[i])

        hand_eye_transform = self._calib.calibrate()
        hand_eye_transform = np.matmul(hand_eye_transform,np.linalg.inv(self._camera_base_transform))
        print 'Hand-eye transform: ', hand_eye_transform
        translation = self.to_translation(hand_eye_transform)
        euler = self.to_euler(hand_eye_transform)
        print "XYZ:", translation
        print "RPY: ", euler


        # calib_data = {'hand_eye_transform':  {'matrix': hand_eye_transform,
        #                                       'xyz': translation,
        #                                       'rpy': euler},
        #               'camera_poses': self._calib._camera_poses,
        #                'ee_poses': self._calib._ee_poses}
        # np.save('hand_eye_calibration_right.npy', calib_data)

    def calibrate(self):
        from getch import getch
        calibrate = False
        print("Press enter to record to next. Add/Remove/Calibrate/Help? (a/r/c/h)")
        while not calibrate:
            
            r = getch()

            

            if r == 'a':
                camera_t, camera_q, _ = self.get_transform(self._camera_frame, self._marker_frame)
                ee_t, ee_q, _ = self.get_transform(self._robot_base_frame, self._ee_frame)

                camera_transform = self.to_transform_matrix(camera_t,camera_q)
                ee_transform = self.to_transform_matrix(ee_t, ee_q)
                print "------------%d Poses---------------"%(len(self._calib._ee_poses),)
                print "Camera transform: ", camera_t, camera_q
                print "Hand transform: ", ee_t, ee_q
                print "---------------------------------"
                self._calib.add_measurement(ee_transform, camera_transform)
                print 'Transform has been added for calibration'
            elif r == 'c':
                calibrate = True
            elif r == 'h':
                print("Press enter to record to next. Add/Remove/Calibrate? (a/r/c/h)")
            elif r == 'r':
                if len(self._calib._ee_poses) > 0:
                    del self._calib._ee_poses[-1]
                    del self._calib._camera_poses[-1]
                    print("Removed last pose")
                else:
                    print("Pose list already empty.")



            elif r in ['\x1b', '\x03']:
                rospy.signal_shutdown("Acquisition finished.")
                break

        if calibrate:
            hand_eye_transform = self._calib.calibrate()

            


            hand_eye_transform = np.matmul(hand_eye_transform,np.linalg.inv(self._camera_base_transform))
            print "Found Transform: ", hand_eye_transform

            translation = self.to_translation(hand_eye_transform)
            euler = self.to_euler(hand_eye_transform)
            print "XYZ:", translation
            print "RPY: ", euler


            calib_data = {'hand_eye_transform':  {'matrix': hand_eye_transform,
                                              'xyz': translation,
                                              'rpy': euler},
                      'camera_poses': self._calib._camera_poses,
                       'ee_poses': self._calib._ee_poses}

            np.save('hand_eye_calibration_right.npy', calib_data)
            np.save(self._save_calib_file,calib_data)
        else:
            print("Quiting. Bye!")







def main():
    calib = HandEyeCalibrator()

    if calib._calib_from_file:
        calib.calibrate_from_file()
    else:
        calib.calibrate()


    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)

    main()
