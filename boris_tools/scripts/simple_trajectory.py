# #! /usr/bin/env python

# # import roslib; roslib.load_manifest('irp6_launch')
# import rospy
# import actionlib

# from trajectory_msgs.msg import *
# from control_msgs.msg import *
# from copy import copy
# import numpy as np

# if __name__ == '__main__':

#     rospy.init_node('simple_trajectory')
#     client = actionlib.SimpleActionClient('lwr/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
#     client.wait_for_server()
#     print 'server ok'
#     goal = FollowJointTrajectoryGoal()
#     goal.trajectory.joint_names = ['lwr_0_joint', 'lwr_1_joint', 'lwr_2_joint', 'lwr_3_joint', 'lwr_4_joint', 'lwr_5_joint', 'lwr_6_joint']

#     waypoints = [[1.825164385348162, 1.803825933603007, 0.000000000000000, 2.008362700195485, 1.321511416992363, -1.519663789792947, -1.768925223209182],
#                 [0.0, 1.803825933603007, 0.000000000000000, 2.008362700195485, 0.0, -1.519663789792947, -1.768925223209182],
#                 [1.825164385348162, 1.203825933603007, 0.000000000000000, 2.008362700195485, 1.321511416992363, -1.519663789792947, -1.768925223209182]
#                 ]
#     times = [5.0, 10.0, 15.0]
    
#     for wpt, t in zip(waypoints,times):
#         point = JointTrajectoryPoint()
#         point.positions = copy(wpt)
#         point.velocities = copy(np.zeros(len(wpt)))
#         point.accelerations = copy(np.zeros(len(wpt)))
#         point.effort = copy(np.zeros(len(wpt)))
#         point.time_from_start = rospy.Duration(t)
#         goal.trajectory.points.append(point)
        
#     goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

#     client.send_goal(goal)

#     client.wait_for_result()