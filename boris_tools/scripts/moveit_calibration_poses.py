#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import tf2_ros
import geometry_msgs.msg
import moveit_msgs.msg
from sensor_msgs.msg import JointState
import tf.transformations as tt
import quaternion_rotation as qr
import math

def get_ee_transfrom(buf, child_frame_id):
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = buf.lookup_transform('world', child_frame_id, rospy.Time(0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("failed to lookup transform")
            rate.sleep()
            continue

    ee_target = geometry_msgs.msg.Pose()
    ee_target.position = trans.transform.translation
    ee_target.orientation = trans.transform.rotation
    q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

    if child_frame_id is 'ee_target':
        qy = tt.quaternion_about_axis(-0.5*math.pi, (0, 1, 0))
        qx = tt.quaternion_about_axis(math.pi, (1, 0, 0))
        q = tt.quaternion_multiply(q, qy)
        q = tt.quaternion_multiply(q, qx)

    if child_frame_id is 'link_6_target':
        pos = qr.quaternion_rotate_vec(q, [0.0, 0.0, -0.16]) # link_6 to ee
        ee_target.position.x += pos[0]
        ee_target.position.y += pos[1]
        ee_target.position.z += pos[2]

        qx = tt.quaternion_about_axis(math.pi, (1, 0, 0))
        q = tt.quaternion_multiply(q, qx)

    ee_target.orientation.x = q[0]
    ee_target.orientation.y = q[1]
    ee_target.orientation.z = q[2]
    ee_target.orientation.w = q[3]

    return ee_target

def move_cartesian_path(waypoints, arm):
    for i in range(10):
        (plan, fraction) = arm.compute_cartesian_path(waypoints,   # waypoints to follow
                                                      0.01,        # eef_step
                                                      0.0)         # jump_threshold
        if fraction < 0.99:
            print("iter=", i, "fraction=", fraction)
        else:
            print("start to move cartesian path")
            arm.execute(plan)
            rospy.sleep(3)
            break

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_commander")


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    arm = moveit_commander.MoveGroupCommander("left_arm")

    arm_initial_pose = arm.get_current_pose().pose
    arm_initial_joints = arm.get_current_joint_values()

    position = [0.385541808244, 0.761554216122, 0.340677951568]
    orientation = [0.795882436029, 0.281121458954 , 0.309208527698, 0.438100399149]

    ee_target = geometry_msgs.msg.Pose()
    ee_target.position.x = position[0]
    ee_target.position.y = position[1]
    ee_target.position.z = position[2]

    ee_target.orientation.x = orientation[0]
    ee_target.orientation.y = orientation[1]
    ee_target.orientation.z = orientation[2]
    ee_target.orientation.w = orientation[3]

    print "Going to pose"

    arm.set_pose_target(ee_target)
    arm.go()


    rospy.sleep(5)

    print "Going back"
    arm.set_pose_target(arm_initial_pose)
    arm.go()
    
    # tf_buffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tf_buffer)

    # pose = get_ee_transfrom(tf_buffer, 'left_arm_7_link')
    

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException, e:
        print(e)
