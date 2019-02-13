#!/usr/bin/env python

import numpy as np
import rospy
import tf
from tf import TransformListener


class TFManager(object):

    def __init__(self):
        
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


if __name__ == '__main__':
    rospy.init_node("test_tf_manager")

    tfm = TFManager()
