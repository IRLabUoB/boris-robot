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
import math


# def move_cartesian_path(waypoints, arm):
#     for i in range(10):
#         (plan, fraction) = arm.compute_cartesian_path(waypoints,   # waypoints to follow
#                                                       0.01,        # eef_step
#                                                       0.0)         # jump_threshold
#         if fraction < 0.99:
#             print("iter=", i, "fraction=", fraction)
#         else:
#             print("start to move cartesian path")
#             arm.execute(plan)
#             rospy.sleep(3)
#             break

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_commander")


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    arm = moveit_commander.MoveGroupCommander("left_hand_arm")

    arm_initial_pose = arm.get_current_pose().pose
    arm_initial_joints = arm.get_current_joint_values()

    position = [0.752, 0.630, -0.10]#[0.385541808244, 0.761554216122, 0.340677951568]
    orientation = [0.120, 0.708, -0.102, 0.689]
#[0.795882436029, 0.281121458954 , 0.309208527698, 0.438100399149]
    # [0.823, 0.640, 0.274]
    #[0.600, -0.395, 0.671, -0.183]

    ## palm down: [0.120, 0.708, -0.102, 0.689]

    ee_target = geometry_msgs.msg.Pose()
    ee_target.position.x = position[0]
    ee_target.position.y = position[1]
    ee_target.position.z = position[2]

    # ee_target.orientation = #arm_initial_pose.orientation
    ee_target.orientation.x = orientation[0]
    ee_target.orientation.y = orientation[1]
    ee_target.orientation.z = orientation[2]
    ee_target.orientation.w = orientation[3]

    print "Going to pose"
    arm.set_pose_target(ee_target)
    arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()


    # print "Going back"
    # arm.set_pose_target(arm_initial_pose)
    # arm.go(wait=True)
    # arm.stop()
    # arm.clear_pose_targets()
    

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException, e:
        print(e)
