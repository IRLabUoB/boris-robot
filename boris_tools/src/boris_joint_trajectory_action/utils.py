import minjerk
import bisect
import numpy as np
import rospy

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

class MinJerkTrajHelper(object):

    def __init__(self):

        self._m_matrix = None
        self._joint_names = None
        self._trajectory_points = None
        self._pnt_times = None
        self._num_points = 0
        self._dimensions_dict = None

    def set_waypoints(self, trajectory_msg):

        self.compute_minjerk_coeff2(trajectory_msg)

    def get_point(self, now = None, start_time = 0):

        if now is None:
            now = rospy.get_time()
        
        now_from_start = now - start_time

        end_time = self._trajectory_points[-1].time_from_start.to_sec()

        point = self._trajectory_points[-1]
        if now_from_start < end_time:
            
            idx = bisect.bisect(self._pnt_times, now_from_start)
    

            #Calculate percentage of time passed in this interval
            if idx >= self._num_points:
                cmd_time = now_from_start - self._pnt_times[0]
                t = 1.0
            elif idx >= 0:
                cmd_time = now_from_start 
                t = cmd_time / (self._pnt_times[-1] - self._pnt_times[0])
            else:
                cmd_time = 0
                t = 0
            

            point = self.get_minjerk_point(self._m_matrix, idx,
                                            t, cmd_time,
                                            self._dimensions_dict)
        #self._command_joints(joint_names, point, start_time, dimensions_dict)
        return point

    def get_point_t(self, t):
        current_time = self._trajectory_points[-1].time_from_start.to_sec()*t + self._trajectory_points[0].time_from_start.to_sec()
        
        #self._command_joints(joint_names, point, start_time, dimensions_dict)
        return self.get_point(now=current_time, start_time=0)

        

    def compute_minjerk_coeff2(self, trajectory_msg):

        joint_names = trajectory_msg.joint_names
        trajectory_points = trajectory_msg.points
        dimensions_dict = self.determine_dimensions(trajectory_points)

        self._joint_names = joint_names
        self._trajectory_points = trajectory_points
        self._dimensions_dict = dimensions_dict

        self._num_points = len(trajectory_points)
        if self._num_points == 0:
            rospy.logerr("Empty Trajectory")
            return None

        # Force Velocites/Accelerations to zero at the final timestep
        # if they exist in the trajectory
        # Remove this behavior if you are stringing together trajectories,
        # and want continuous, non-zero velocities/accelerations between
        # trajectories
        if dimensions_dict['velocities']:
            trajectory_points[-1].velocities = [0.0] * len(joint_names)
        if dimensions_dict['accelerations']:
            trajectory_points[-1].accelerations = [0.0] * len(joint_names)


        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]

        point_duration = [pnt_times[i+1] - pnt_times[i] for i in range(len(pnt_times)-1)]
        m_matrix = self.compute_minjerk_coeff(joint_names,
                                                trajectory_points,
                                                point_duration,
                                                dimensions_dict)

        self._pnt_times = pnt_times
        # setting m_matrix
        self._m_matrix = m_matrix
        




    def compute_minjerk_coeff(self, joint_names, trajectory_points, point_duration, dimensions_dict):
            # Compute Full Minimum Jerk Curve
            num_joints = len(joint_names)
            num_traj_pts = len(trajectory_points)
            num_traj_dim = sum(dimensions_dict.values())
            num_m_values = len(['a0', 'a1', 'a2', 'a3', 'a4', 'a5', 'tm'])
            m_matrix = np.zeros(shape=(num_joints, num_traj_dim, num_traj_pts-1, num_m_values))
            for jnt in xrange(num_joints):
                traj_array = np.zeros(shape=(len(trajectory_points), num_traj_dim))
                for idx, point in enumerate(trajectory_points):
                    current_point = list()
                    current_point.append(point.positions[jnt])
                    if dimensions_dict['velocities']:
                        current_point.append(point.velocities[jnt])
                    if dimensions_dict['accelerations']:
                        current_point.append(point.accelerations[jnt])
                    traj_array[idx, :] = current_point
                m_matrix[jnt, :, :, :] = minjerk.minjerk_coefficients(traj_array, point_duration)
            return m_matrix


    def get_minjerk_point(self, m_matrix, idx, t, cmd_time, dimensions_dict):
            pnt = JointTrajectoryPoint()
            pnt.time_from_start = rospy.Duration(cmd_time)
            num_joints = m_matrix.shape[0]
            pnt.positions = [0.0] * num_joints
            if dimensions_dict['velocities']:
                pnt.velocities = [0.0] * num_joints
            if dimensions_dict['accelerations']:
                pnt.accelerations = [0.0] * num_joints
            for jnt in range(num_joints):
                m_point = minjerk.minjerk_point(m_matrix[jnt, :, :, :], idx, t)
                # Positions at specified time
                pnt.positions[jnt] = m_point[0]
                # Velocities at specified time
                if dimensions_dict['velocities']:
                    pnt.velocities[jnt] = m_point[1]
                # Accelerations at specified time
                if dimensions_dict['accelerations']:
                    pnt.accelerations[jnt] = m_point[-1]
            return pnt

    def determine_dimensions(self, trajectory_points):
            # Determine dimensions supplied
            position_flag = True
            velocity_flag = (len(trajectory_points[0].velocities) != 0 and
                            len(trajectory_points[-1].velocities) != 0)
            acceleration_flag = (len(trajectory_points[0].accelerations) != 0 and
                                len(trajectory_points[-1].accelerations) != 0)
            return {'positions':position_flag,
                    'velocities':velocity_flag,
                    'accelerations':acceleration_flag}