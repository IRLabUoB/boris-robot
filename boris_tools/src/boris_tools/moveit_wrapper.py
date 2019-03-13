import rospy

import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg

import sys

class MoveitWrapper(object):


    def __init__(self):

        self._ready = False

        self._display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)


    def init_moveit_commander(self):
        """
        Initialises moveit commander interface
        Warning: Needs to be called before before rospy.init_node()
        """
        moveit_commander.roscpp_initialize(sys.argv)


    def setup(self):

        """
        Initialises moveit interfaces for all move groups
        """

        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()

        group_names = self._robot.get_group_names()

        self._move_groups = {}

        for group_name in group_names:
            self._move_groups[group_name] = moveit_commander.MoveGroupCommander(group_name)
            
            # safe values
            self._move_groups[group_name].set_max_velocity_scaling_factor(0.25)
            self._move_groups[group_name].set_max_acceleration_scaling_factor(0.25)


        self._ready = True

    def scene(self):
        return self._scene
    
    def robot(self):
        return self._robot

    def is_ready(self):
        """
        Return whether or not this wrapper obeject is ready to be used
        @rtype: bool
        @return: Return whether or not this wrapper obeject is ready to be used
        """
        return self._ready

    def get_group_names(self):

        """
        Return group names for this robot
        @rtype: [str]
        @return: Return a moveit RobotState message describing the current state of the robot
        """

        assert self._ready

        return self._robot.get_group_names()

    def get_current_state(self):
        """
        Return a moveit RobotState message describing the current state of the robot
        see: http://docs.ros.org/kinetic/api/moveit_commander/html/annotated.html
        @rtype: RobotState
        @return: Return a moveit RobotState message describing the current state of the robot
        """

        assert self._ready

        return self._robot.get_current_state()

    def get_current_pose(self, group_name):
        """
        Return a geometry_msgs.msg.Pose message describing the current end-effector pose of the move group
        see: http://docs.ros.org/kinetic/api/moveit_commander/html/annotated.html
        @type group_name: str
        @param group_name: move group name  (see get_group_names())
        @rtype: geometry_msgs.msg.Pose
        @return: Return a geometry_msgs.msg.Pose message describing the current end-effector pose of the move group
        """
        assert self._ready

        return self._move_groups[group_name].get_current_pose().pose

    def get_current_joint_values(self, group_name):
        """
        Return a list [float] describing the current joint angles of the group
        see: http://docs.ros.org/kinetic/api/moveit_commander/html/annotated.html
        @type group_name: str
        @param group_name: move group name  (see get_group_names())
        @rtype: [float]
        @return: Return a list [float] message describing the current joint angles of the group
        """
        assert self._ready

        return self._move_groups[group_name].get_current_joint_values()

    def clear_pose_targets(self, group_name):
        """
        Clears pose and joint value targets for specified move group
        @type group_name: str
        @param group_name: move group name  (see get_group_names())
        """

        assert self._ready

        self._move_groups[group_name].clear_pose_targets()

    def set_max_velocity_scaling_factor(self, group_name, scale):
        """
        Specify velocity scaling factor for given group
        @type group_name: str
        @param group_name: move group name  (see get_group_names())
        @type scale: float 
        @param scale: velocity scale. Allowed values are in (0,1]. 
        """

        assert self._ready

        self._move_groups[group_name].set_max_velocity_scaling_factor(scale)

    def set_max_acceleration_scaling_factor(self, group_name, scale):
        """
        Specify acceleration scaling factor for given group
        @type group_name: str
        @param group_name: move group name  (see get_group_names())
        @type scale: float 
        @param scale: acceleration scale. Allowed values are in (0,1].
        """

        assert self._ready

        self._move_groups[group_name].set_max_acceleration_scaling_factor(scale)

    def set_joint_value_target(self, group_name, joint_angles_target):
        """
        Specify a target joint configuration for the group.
        @type group_name: str
        @param group_name: move group name  (see get_group_names())
        @type joint_angles_target: dict, [float] or JointState message
        @param joint_angles_target: target joint angles specifying a target joint configuration for the group.
        """

        assert self._ready

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        # see example: 
        # https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
        self.clear_pose_targets(group_name)

        self._move_groups[group_name].set_joint_value_target(joint_angles_target)

    def set_pose_target(self, group_name, pose_target):
        """
        Set cartesian space pose target
        @type group_name: str
        @param group_name: move group name  (see get_group_names())
        @type pose_target: geometry_msgs.msg.Pose
        @param pose_target: geometry_msgs.msg.Pose object representing target cartesian pose

        """

        assert self._ready

        self._move_groups[group_name].set_pose_target(pose_target)

    def plan(self, group_name, display = False, target = None):
        """
        Return a motion plan (a RobotTrajectory) to the set goal state (or specified by the joints argument) 
        for the specific group name. 
        Plan will be send for display in RVIZ.
        If target is None then:
        1) One must have called set_pose_target beforehand if planning in cartesian space 
        2) One must have called set_pose_target beforehand if planning in joint space 
        @type group_name: str
        @param group_name: move group name  (see get_group_names())
        @type display: bool
        @param display: boolean specifying whether or not to send the planned path for RVIZ display
        @type target: [float] or geometry_msgs.msg.Pose
        @param target: joint angles as optional target configuration
        @rtype: RobotTrajectory 
        """

        assert self._ready
        
        plan = self._move_groups[group_name].plan(joints=target)

        if display:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()

            display_trajectory.trajectory_start = self._robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            self._display_trajectory_publisher.publish(display_trajectory)

        return plan

    def execute(self, group_name, plan_msg, wait=True):

        """
        Execute a previously planned path (RobotTrajectory)
        for the specific group name. 
        @type group_name: str
        @param group_name: move group name  (see get_group_names())
        @type plan_msg: RobotTrajectory
        @param plan_msg: RobotTrajectory planned path to be executed
        @type wait: bool
        @param wait: Execute synchronously (method call will block) or asynchronously (non blocking call)
        @rtype: MoveItErrorCode 
        """

        assert self._ready


        ret = self._move_groups[group_name].execute(plan_msg=plan_msg, wait=wait)

        if wait:
            self._move_groups[group_name].stop()
            self._move_groups[group_name].clear_pose_targets()

        return ret


    def compute_cartesian_path(self, group_name, waypoints):

        """
        Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints. 
        Configurations are computed for every eef_step meters; The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resulting path; 
        Kinematic constraints for the path given by path_constraints will be met for every point along the trajectory, if they are not met, a partial solution will be returned. 
        The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory. 

        for the specific group name. 
        @type group_name: str
        @param group_name: move group name  (see get_group_names())
        @type waypoints: [geometry_msgs.msg.Pose]
        @param waypoints: Waypoints to be followed in the computed path 
        @rtype: (RobotTrajectory, float)
        """

        assert self._ready

        (plan, fraction) = self._move_groups[group_name].compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold

        return plan, fraction
        


        


        


        