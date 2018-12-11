

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

from boris_robot import BorisRobot

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
        # self._goal.trajectory.joint_names = self._joint_names
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        finish = self._client.wait_for_result(timeout=rospy.Duration(timeout))
        # result = (self._client.get_result().error_code == 0)
        print finish

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self._joint_names


def parse_file(filename):
    """
    Parses input file into list of waypoints format
    @param filename: input filename
    @return: trajectory, joint_names
    """
    #open recorded file
    with open(filename, 'r') as f:
        lines = f.readlines()
    #read joint names specified in file
    csv_fields = lines[0].rstrip().split(',')
    joint_names = csv_fields[1:]

    trajectory = []
    for l in lines[1:]:
        sepl = l.split(',')
        values = [float(x) for x in sepl]

        trajectory.append(values)


    return trajectory, joint_names
    #parse joint names for right limb
    # for name in joint_names:
    #     if self.limb == name[:-3]:
    #         self.goal.trajectory.joint_names.append(name)

def main():
    
    rospy.init_node('aml_trajectory_player')

    trajectory, joint_names = parse_file('test_traj.csv')
    # joint_names = ['left_arm_0_joint', 'left_arm_1_joint', 'left_arm_2_joint', 'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint', 'left_hand_synergy_joint', 'left_hand_thumb_abd_joint', 'left_hand_thumb_inner_joint', 'left_hand_thumb_inner_joint_mimic', 'left_hand_thumb_outer_joint', 'left_hand_thumb_outer_joint_mimic', 'left_hand_index_abd_joint', 'left_hand_index_inner_joint', 'left_hand_index_inner_joint_mimic', 'left_hand_index_middle_joint', 'left_hand_index_middle_joint_mimic', 'left_hand_index_outer_joint', 'left_hand_index_outer_joint_mimic', 'left_hand_middle_abd_joint', 'left_hand_middle_inner_joint', 'left_hand_middle_inner_joint_mimic', 'left_hand_middle_middle_joint', 'left_hand_middle_middle_joint_mimic', 'left_hand_middle_outer_joint', 'left_hand_middle_outer_joint_mimic', 'left_hand_ring_abd_joint', 'left_hand_ring_inner_joint', 'left_hand_ring_inner_joint_mimic', 'left_hand_ring_middle_joint', 'left_hand_ring_middle_joint_mimic', 'left_hand_ring_outer_joint', 'left_hand_ring_outer_joint_mimic', 'left_hand_little_abd_joint', 'left_hand_little_inner_joint', 'left_hand_little_inner_joint_mimic', 'left_hand_little_middle_joint', 'left_hand_little_middle_joint_mimic', 'left_hand_little_outer_joint', 'left_hand_little_outer_joint_mimic', 'right_arm_0_joint', 'right_arm_1_joint', 'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint', 'right_arm_6_joint', 'right_hand_synergy_joint', 'right_hand_thumb_abd_joint', 'right_hand_thumb_inner_joint', 'right_hand_thumb_inner_joint_mimic', 'right_hand_thumb_outer_joint', 'right_hand_thumb_outer_joint_mimic', 'right_hand_index_abd_joint', 'right_hand_index_inner_joint', 'right_hand_index_inner_joint_mimic', 'right_hand_index_middle_joint', 'right_hand_index_middle_joint_mimic', 'right_hand_index_outer_joint', 'right_hand_index_outer_joint_mimic', 'right_hand_middle_abd_joint', 'right_hand_middle_inner_joint', 'right_hand_middle_inner_joint_mimic', 'right_hand_middle_middle_joint', 'right_hand_middle_middle_joint_mimic', 'right_hand_middle_outer_joint', 'right_hand_middle_outer_joint_mimic', 'right_hand_ring_abd_joint', 'right_hand_ring_inner_joint', 'right_hand_ring_inner_joint_mimic', 'right_hand_ring_middle_joint', 'right_hand_ring_middle_joint_mimic', 'right_hand_ring_outer_joint', 'right_hand_ring_outer_joint_mimic', 'right_hand_little_abd_joint', 'right_hand_little_inner_joint', 'right_hand_little_inner_joint_mimic', 'right_hand_little_middle_joint', 'right_hand_little_middle_joint_mimic', 'right_hand_little_outer_joint', 'right_hand_little_outer_joint_mimic', 'head_neck_pitch_joint', 'head_neck_roll_joint', 'head_neck_yaw_joint', 'head_head_tilt_joint', 'head_eyes_tilt_joint', 'head_left_eye_joint', 'head_right_eye_joint']
    traj = Trajectory(robot_part='left_arm')
    traj.clear()
    # traj_hand = Trajectory(robot_part='left_hand')
    robot = BorisRobot()
    rate = rospy.Rate(10)
    rate.sleep()
    current = [robot.joint_angles()[jn] for jn in joint_names]
    print current
    traj.add_point(current,0.0)
    # print robot.joint_angles()
    # traj_hand.add_point([0.0],0.0)
    
    for wpt in trajectory:
        # print len(wpt)
        # print wpt[0]
        traj.add_point(wpt[1:8], wpt[0])
        # print wpt[1:8], "->", wpt[0]
        # print wpt[0]*0.1, "-> ", wpt[8:9]
    # traj_hand.add_point([0.0], 2.0)
    # print trajectory[-1][8:9], trajectory[-1][0]*0.01
    
    rospy.on_shutdown(traj.stop)

    # traj.add_point([0.0]*7,float(0.0))
    # traj.add_point([0.5,0.5,0.5,0.5,0.5,0.5,0.5], float(2.0))
    # traj.start()
    # result = traj.wait(timeout=trajectory[-1][0])

    result = True
    loop_cnt = 0
    max_cnt = 100
    
    while (result == True and loop_cnt <= max_cnt
           and not rospy.is_shutdown()):
        print("Playback loop %d of %d" % (loop_cnt, max_cnt,))
        traj.start()
        
        result = traj.wait(timeout=trajectory[-1][0])
        
        # # traj_hand.start()
        # # result = traj_hand.wait(timeout=trajectory[-1][0])
        # loop_cnt = loop_cnt + 1
        # print result

        rate.sleep()


if __name__ == '__main__':
    main()