#!/usr/bin/env python

import rospy
import geometry_msgs
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

import sys
import copy
import tf2_ros
import tf.transformations as tt
import math
import quaternion_rotation as qr

def get_mock_gravity(buf, child_frame_id):
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = buf.lookup_transform('world', child_frame_id, rospy.Time(0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("failed to lookup transform")
            print e
            rate.sleep()
            continue

    ee_target = geometry_msgs.msg.Pose()
    ee_target.position = trans.transform.translation
    ee_target.orientation = trans.transform.rotation
    q = [-trans.transform.rotation.x, -trans.transform.rotation.y, -trans.transform.rotation.z, trans.transform.rotation.w]

    gravity = qr.quaternion_rotate_vec(q, [0.0, 0.0, -9.8]) 

    return gravity

def main():

    rospy.init_node("mock_imu_sensor")

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    sensor_rate = rospy.get_param("rate",100)
    imu_topic = rospy.get_param("imu_topic","left_arm/imu_sensor")
    frame_id = rospy.get_param("frame_id","left_ft_sensor")
    print frame_id

    rate = rospy.Rate(sensor_rate)
    
    print("sensor_rate:", sensor_rate)
    

    imu_publisher = rospy.Publisher(imu_topic, Imu, queue_size=0)
    imu_publisher_scalled = rospy.Publisher("left_arm/imu_sensor_scalled", Imu, queue_size=0)

    # Polling from network
    while not rospy.is_shutdown():

        gravity = get_mock_gravity(tf_buffer,frame_id)

        # print gravity

        imu_msg = Imu()
        imu_msg.linear_acceleration = Vector3(*gravity)
       
        time_stamp = rospy.Time.now()
        imu_msg.header.stamp.secs = time_stamp.secs
        imu_msg.header.stamp.nsecs = time_stamp.nsecs

        imu_msg.header.frame_id = frame_id

        imu_publisher.publish(imu_msg)

        imu_msg.linear_acceleration.x *= 0.05
        imu_msg.linear_acceleration.y *= 0.05
        imu_msg.linear_acceleration.z *= 0.05
        imu_publisher_scalled.publish(imu_msg)

        rate.sleep()



if __name__ == '__main__':
    main()



