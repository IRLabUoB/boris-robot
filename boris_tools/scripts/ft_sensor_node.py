#!/usr/bin/env python

from __future__ import print_function
import rospy

import geometry_msgs
from geometry_msgs.msg import WrenchStamped, Vector3
from sensor_msgs.msg import Imu

from boris_tools.network_ft_sensor import NetFTSensor

import numpy as np
import copy

GRIPPER_MASS = 0.88147962180902968
GRIPPER_COM = np.array([-0.0007626476070273522, -0.0015036809041290501, 0.10747368058393117])
BIAS = np.array([-0.6596273431449243, 0.3271468132085467, -5.0491181111789025, 0.004470535053033756,-0.03623978486740379, -0.008814876887642405])
COMPENSATED_BIAS = np.array([-0.06679442, -0.12326067, -0.24217093,0.04683272, 0.0758901, -0.00438831])

RECOMPUTE_COMP_BIAS = False

class IMUSensor(object):

        def __init__(self):
                
                self._sub = rospy.Subscriber("/left_arm/imu_sensor", Imu, self.imu_callback)
                self._accel = None

        def imu_callback(self, imu_msg):

                self._accel = imu_msg.linear_acceleration

        def get_accel(self):
                if self._accel is None:
                        return np.zeros(3)

                return copy.deepcopy(np.array([self._accel.x, self._accel.y, self._accel.z]))



def make_wrench_msg(force, torque, frame_id, stamp):
        wrench = WrenchStamped()

    
        wrench.wrench.force = Vector3(*force)
        wrench.wrench.torque = Vector3(*torque)

        time_stamp = rospy.Time.from_sec(stamp)

        wrench.header.stamp.secs = time_stamp.secs
        wrench.header.stamp.nsecs = time_stamp.nsecs

        wrench.header.frame_id = frame_id

        return wrench

def main():

    rospy.init_node("network_ft_sensor_node")

    sensor_rate = rospy.get_param("rate",1000)
    ft_topic_raw = rospy.get_param("ft_topic_raw","left_arm/ft_sensor_raw")
    ft_topic = rospy.get_param("ft_topic","left_arm/ft_sensor")
    ft_topic_pred = rospy.get_param("ft_topic_pred","left_arm/ft_sensor_pred")
    frame_id = rospy.get_param("frame_id","left_ft_sensor")
    host_ip = rospy.get_param("host_ip","10.0.11.163")

    rate = rospy.Rate(sensor_rate)
    
#     print("sensor_rate:", sensor_rate)
    
    ft_sensor = NetFTSensor(ip=host_ip)

    ft_publisher_raw = rospy.Publisher(ft_topic_raw, WrenchStamped, queue_size=0)
    ft_publisher = rospy.Publisher(ft_topic, WrenchStamped, queue_size=0)
    ft_publisher_pred = rospy.Publisher(ft_topic_pred, WrenchStamped, queue_size=0)

    imu_sensor = IMUSensor()

    compensated_bias = np.zeros(6) if RECOMPUTE_COMP_BIAS else COMPENSATED_BIAS
    bias_readings = 0
    computed_bias = False or not RECOMPUTE_COMP_BIAS

    # Polling from network
    while not rospy.is_shutdown():

        force, torque, stamp = ft_sensor.read_wrench()

        # Raw readings
        wrench_stamped_raw = make_wrench_msg(force, torque, frame_id, stamp)
        ft_publisher_raw.publish(wrench_stamped_raw)

        # Predicted readings
        
        r = GRIPPER_COM

        # Accel vector (mainly the direction of gravity) is given in local frame of the f/t sensor
        # At the moment we are using only a mock IMU that gives the locally measured gravit vector only
        accel = imu_sensor.get_accel()

        if np.isclose(np.linalg.norm(accel),0.0):
                continue

        force_pred = accel*GRIPPER_MASS + BIAS[:3]
        torque_pred = np.cross(r, force_pred) + BIAS[3:]


        wrench_stamped_pred = make_wrench_msg(force_pred, torque_pred, frame_id, stamp)

        ft_publisher_pred.publish(wrench_stamped_pred)

        # Calibrated readings
        
        calibrated_force = force - force_pred 
        calibrated_torque = torque - torque_pred 

        # Additional bias term
        if bias_readings < 10000 and RECOMPUTE_COMP_BIAS:
                compensated_bias[:3] += calibrated_force
                compensated_bias[3:] += calibrated_torque
                bias_readings += 1

                if bias_readings >= 10000:
                        computed_bias = True
                        compensated_bias /= bias_readings
                        print("Bias: ",compensated_bias)
                        raw_input("wait")
        
        if computed_bias:
                calibrated_force -= compensated_bias[:3]
                calibrated_torque -= compensated_bias[3:]

        wrench_calibrated = make_wrench_msg(calibrated_force, calibrated_torque, frame_id, stamp)
        
        ft_publisher.publish(wrench_calibrated)

        # print("------")
        # print(np.linalg.norm(calibrated_force))
        # print(np.linalg.norm(calibrated_torque))
        # print("------")


        rate.sleep()



if __name__ == '__main__':
    main()



