import rospy

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)
from sensor_msgs.msg import JointState

import copy

def parse_trajectory_file(filename):
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

    waypoints = []
    for l in lines[1:]:
        sepl = l.split(',')
        values = [float(x) for x in sepl]

        waypoints.append(values)


    return waypoints, joint_names

def make_ros_trajectory_msg(waypoints, joint_names, index_map=(1,-1)):
    """
    Converts waypoints and joint_names to trajectory_msgs.msg.JointTrajectory
    @param waypoints: input waypoints [timestamp,joint_0, ..., joint_N]
    @param joint_names: input trajectory names [joint_name_0, ..., joint_name_N]
    @return: converted trajectory_msgs.msg.JointTrajectory
    """
    dur = []
    traj = JointTrajectory()
    traj.joint_names = joint_names
    

    for wpt in waypoints:
        point = JointTrajectoryPoint()
        for cmd in wpt[index_map[0]:index_map[1]]:
            # max_vel = self._robot_joint_limits[name]['max_velocity']
            # dur.append(max(abs(cmd - pos) / max_vel, self._min_traj_dur))
            point.positions.append(cmd)
        
        # print wpt[0]*1.1, "->", wpt[1:8]
        point.time_from_start = rospy.Duration(wpt[0])

        traj.points.append(point)

    return traj

def trajectory_point2joint_state(joint_trajectory_point):

    """
    Converts JointTrajectoryPoint to JointState
    @param joint_trajectory_point: input JointTrajectoryPoint
    @return: converted JointState
    """

    joint_state = JointState()
    joint_state.position = copy.deepcopy(joint_trajectory_point.positions)
    joint_state.velocity = copy.deepcopy(joint_trajectory_point.velocities)
    joint_state.effort = copy.deepcopy(joint_trajectory_point.effort)
    
    return joint_state


def make_cartesian_trajectory(waypoints, index_map=(1,-1), fk_func=None):

    assert fk_func is not None

    cartesian_trajectory = []
    for wpt in waypoints:

        cartesian_trajectory.append(fk_func(wpt[index_map[0]:index_map[1]]))

    
    return cartesian_trajectory


# def make_cartesian_trajectory_poseMsgs(waypoints, index_map=(1,-1), fk_func=None):

#     assert fk_func is not None

#     cartesian_trajectory = []
#     for wpt in waypoints:

#         cartesian_trajectory.append(fk_func(wpt[index_map[0]:index_map[1]]))

    
#     return cartesian_trajectory


