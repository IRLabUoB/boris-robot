

import sys
from copy import copy
import rospy


from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)

import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg

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
def make_trajectory(waypoints, joint_names):
    dur = []
    traj = JointTrajectory()
    traj.joint_names = joint_names
    

    for wpt in waypoints:
        point = JointTrajectoryPoint()
        for cmd in wpt[1:8]:
            # max_vel = self._robot_joint_limits[name]['max_velocity']
            # dur.append(max(abs(cmd - pos) / max_vel, self._min_traj_dur))
            point.positions.append(cmd)
        
        # print wpt[0]*1.1, "->", wpt[1:8]
        point.time_from_start = rospy.Duration(wpt[0])

        traj.points.append(point)

    return traj

def make_trajectory_hand(waypoints, joint_names):
    dur = []
    traj = JointTrajectory()
    traj.joint_names = joint_names
    

    for wpt in waypoints:
        point = JointTrajectoryPoint()
        for cmd in wpt[8:9]:
            # max_vel = self._robot_joint_limits[name]['max_velocity']
            # dur.append(max(abs(cmd - pos) / max_vel, self._min_traj_dur))
            point.positions.append(cmd)
        
        print wpt[0]*1.1, "->", wpt[1:8]
        point.time_from_start = rospy.Duration(wpt[0])

        traj.points.append(point)

    return traj

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('aml_trajectory_player')
    cmd_topic = "/left_arm/joint_trajectory_controller/command"
    cmd_pub = rospy.Publisher(cmd_topic,
                                    JointTrajectory,
                                    queue_size=1)
    hand_cmd_pub = rospy.Publisher("/left_hand/joint_trajectory_controller/command",
                                    JointTrajectory,
                                    queue_size=1)

    # trajectory = [-2.02830171585, 1.04696202278, 1.55436050892, -1.44723391533, -0.815707325935, 0.387918055058, -2.68925023079]
    joint_names = rospy.get_param("left_arm/joints")
    hand_joint_names = rospy.get_param("left_hand/joints")
    trajectory, _ = parse_file('spatula_trajectory.csv')

    traj = make_trajectory(trajectory,joint_names)
    traj_hand = make_trajectory_hand(trajectory,hand_joint_names)

   


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    arm = moveit_commander.MoveGroupCommander("left_arm")

    try:
        goal = trajectory[0][1:8]
        arm.set_joint_value_target(goal)
        arm.set_max_velocity_scaling_factor(0.35)
        arm.set_max_acceleration_scaling_factor(0.35)
        arm.go(wait=True)
        arm.stop()
    except:
        print("Not possible to go to waypoint. Try another one")

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        c = raw_input("send: ")
        if c == "y":
            hand_cmd_pub.publish(traj_hand)
            cmd_pub.publish(traj)
        rate.sleep()


if __name__ == '__main__':
    main()