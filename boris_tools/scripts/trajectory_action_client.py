import sys
from copy import copy
import rospy
import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)


class Trajectory(object):
    def __init__(self, robot_part="left_hand", joint_names=None):
        if joint_names is None:
            self._joint_names = rospy.get_param("%s/joints"%(robot_part,))
        else:
            self._joint_names = joint_names
            print self._joint_names

        print self._joint_names
        ns = "/%s/joint_trajectory_controller/"%(robot_part,)
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

    
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        

        rospy.loginfo("Trying to connect to trajectory server")
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        else:
            rospy.loginfo("Has connected to trajectory server")
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)
        

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._goal.trajectory.joint_names = self._joint_names
        self._client.send_goal(self._goal)
        rospy.loginfo("Sent trajectory goal")

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        finish = self._client.wait_for_result(timeout=rospy.Duration(timeout))
        # result = (self._client.get_result().error_code == 0)
        print finish
        print self.result()

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self._joint_names
