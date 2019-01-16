
#!/usr/bin/env python

from __future__ import print_function

import rospy


from geometry_msgs.msg import WrenchStamped, Vector3

from boris_tools.network_ft_sensor import NetFTSensor

def main():

    rospy.init_node("network_ft_sensor_node")

    sensor_rate = rospy.get_param("rate",1000)
    ft_topic = rospy.get_param("ft_topic","left_arm/ft_sensor")
    frame_id = rospy.get_param("frame_id","left_arm_7_link")
    host_ip = rospy.get_param("host_ip","10.0.11.163")

    rate = rospy.Rate(sensor_rate)
    
    print("sensor_rate:", sensor_rate)
    
    ft_sensor = NetFTSensor(ip=host_ip)

    ft_publisher = rospy.Publisher(ft_topic, WrenchStamped, queue_size=1)

    # Polling from network
    while not rospy.is_shutdown():

        force, torque, stamp = ft_sensor.read_wrench()

        wrench_stamped = WrenchStamped()
        wrench_stamped.wrench.force = Vector3(*force)
        wrench_stamped.wrench.torque = Vector3(*torque)

        time_stamp = rospy.Time.from_sec(stamp)
        wrench_stamped.header.stamp.secs = time_stamp.secs
        wrench_stamped.header.stamp.nsecs = time_stamp.nsecs

        wrench_stamped.header.frame_id = frame_id

        ft_publisher.publish(wrench_stamped)

        rate.sleep()



if __name__ == '__main__':
    main()



