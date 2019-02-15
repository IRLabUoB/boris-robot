#!/usr/bin/env python
from copy import deepcopy
import warnings

import rospy
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)

from boris_tools.boris_kinematics import boris_kinematics


from cartesian_imp_commander import CartesianImpedanceCommander
from joint_imp_commander import JointImpedanceCommander

class BorisRobot(object):
    

    def __init__(self, name='boris', moveit_wrapper = None):
        """
        Constructs an instance of Boris Robot.

        @type name: str
        @param name: robot name
        @type moveit_wrapper: boris_tools.moveit_wrapper.MoveitWrapper
        @moveit_wrapper: a pre initialised instance of MoveitWrapper

        """

        self.name = name

        self._joint_names_map = {"left_arm": rospy.get_param("left_arm/joints"),
                                "left_hand": rospy.get_param("left_hand/joints"),
                                "right_arm": rospy.get_param("right_arm/joints"),
                                "right_hand": rospy.get_param("right_hand/joints"),
                                "head": rospy.get_param("head/joints")}
        joint_names = []
        joint_names = rospy.get_param("left_arm/joints")
        joint_names += rospy.get_param("left_hand/joints")
        joint_names += rospy.get_param("right_arm/joints")
        joint_names += rospy.get_param("right_hand/joints")
        joint_names += rospy.get_param("head/joints")

        self._joint_angle = dict()
        self._joint_velocity = dict()
        self._joint_effort = dict()
        self._cartesian_pose = dict()
        self._cartesian_velocity = dict()
        self._cartesian_effort = dict()
        self._joint_names = joint_names
        self._joint_names_map["all"] = joint_names

        joint_state_topic = "/joint_states"
        self._joint_state_sub = rospy.Subscriber(
                                        joint_state_topic,
                                        JointState,
                                        self._on_joint_states,
                                        queue_size=1,
                                        tcp_nodelay=True)

        queue_size = None
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")

            left_arm_cmd_topic = "/left_arm/joint_trajectory_controller/command"
            self.left_arm_cmd_pub_ = rospy.Publisher(
                                            left_arm_cmd_topic,
                                            JointTrajectory,
                                            queue_size=queue_size)

            right_arm_cmd_topic = "/right_arm/joint_trajectory_controller/command"
            self.right_arm_cmd_pub_ = rospy.Publisher(
                                            right_arm_cmd_topic,
                                            JointTrajectory,
                                            queue_size=queue_size)

            left_hand_cmd_topic = "/left_hand/joint_trajectory_controller/command"
            self.left_hand_cmd_pub_ = rospy.Publisher(
                                            left_hand_cmd_topic,
                                            JointTrajectory,
                                            queue_size=queue_size)


        self.cmd_map_ = {
            "left_arm" : self.left_arm_cmd_pub_,
            "right_arm" : self.right_arm_cmd_pub_,
            "left_hand" : self.left_hand_cmd_pub_,
        }


        self._moveit_wrapper = moveit_wrapper
        self._has_moveit = self._moveit_wrapper is not None and self._moveit_wrapper.is_ready()


        self._commander = None
        self._mode = "position"



    def _on_joint_states(self, msg):
        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_angle[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]
        
        # joint_angles = np.array(joint_state_msg.position)
        # joint_velocities = np.array(joint_state_msg.velocity)
        # joint_efforts = np.array(joint_state_msg.effort)
        # joint_names = joint_state_msg.names
    
    def joint_angles(self):
        """
        Return all joint angles.
        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to angle (rad) Values
        """
        return deepcopy(self._joint_angle)

    def joint_angle(self, joint):
        """
        Return the requested joint angle.
        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: angle in radians of individual joint
        """
        return self._joint_angle[joint]


    def joint_velocities(self):
        """
        Return all joint velocities.
        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to velocity (rad/s) Values
        """
        return deepcopy(self._joint_velocity)


    def joint_velocity(self, joint):
        """
        Return the requested joint velocity.
        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: velocity in radians/s of individual joint
        """
        return self._joint_velocity[joint]
    
    def joint_efforts(self):
        """
        Return all joint efforts.
        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to effort (Nm) Values
        """
        return deepcopy(self._joint_effort)

    def joint_effort(self, joint):
        """
        Return the requested joint effort.
        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: effort in Nm of individual joint
        """
        return self._joint_effort[joint]


    def joint_names(self, limb_name="all"):
        """
        Return the names of the joints for the specified limb.
        @rtype: [str]
        @return: ordered list of joint names for Boris
        """
        return self._joint_names_map.get(limb_name,self._joint_names)


    def goto_with_moveit(self, limb_name, joint_values):


        if self._has_moveit:

            # Go to first waypoint
            self._moveit_wrapper.set_joint_value_target(limb_name, joint_values)
            plan = self._moveit_wrapper.plan(group_name=limb_name, display=True)
            self._moveit_wrapper.execute(group_name=limb_name, plan_msg=plan)

    def get_moveit_plan(self, limb_name, joint_values):


        if self._has_moveit:

            # Go to first waypoint
            self._moveit_wrapper.set_joint_value_target(limb_name, joint_values)
            plan = self._moveit_wrapper.plan(group_name=limb_name, display=True)

            return plan
        else:
            return None


    def follow_trajectory(self, limb_name, joint_trajectory_msg, first_waypoint_moveit=True):
        """
        Using topic interface for trajectory tracking (optionally going to the first waypoint with moveit)
        Send joint trajectory message to be executed by the specified limb
        @type limb_name: str
        @param limb_name: name of the limb which should follow the trajectory 
        options: ("left_arm", "right_arm", "left_hand", "right_hand")
        @type first_waypoint_moveit: bool
        @param first_waypoint_moveit: whether or not to use moveit to take the robot tot he first waypoint
        """

        if first_waypoint_moveit and self._has_moveit:
            trajectory_point = joint_trajectory_msg.points[0]

            # Go to first waypoint
            self.goto_with_moveit(limb_name, trajectory_point.positions)


        self.cmd_map_[limb_name].publish(joint_trajectory_msg)
    
    # def follow_cart_imp_trajectory(self, limb_name, joint_trajectory_msg, cartesian_trajectory, first_waypoint_moveit=True):
    #     """
    #     Using topic interface for trajectory tracking
    #     Send joint trajectory message to be executed by the specified limb
    #     @type limb_name: str
    #     @param limb_name: name of the limb which should follow the trajectory 
    #     options: ("left_arm", "right_arm", "left_hand", "right_hand")
    #     @type first_waypoint_moveit: bool
    #     @param first_waypoint_moveit: whether or not to use moveit to take the robot tot he first waypoint
    #     """

    #     if first_waypoint_moveit and self._has_moveit:
    #         trajectory_point = joint_trajectory_msg.points[0]

    #         # Go to first waypoint
    #         self._moveit_wrapper.set_joint_value_target(limb_name, trajectory_point.positions)
    #         plan = self._moveit_wrapper.plan(group_name=limb_name, display=True)
    #         self._moveit_wrapper.execute(group_name=limb_name, plan_msg=plan)

    #     self.cmd_map_[limb_name].publish(joint_trajectory_msg)
        
    def stop_trajectory(self, limb_name):
        """
        Stop execution of all queued trajectories for specified limb
        @param limb_name: name of the limb which should follow the trajectory 
        options: ("left_arm", "right_arm", "left_hand", "right_hand")
        """

        # sending empty trajectory to stop all trajectories
        # see preemption policy: http://wiki.ros.org/joint_trajectory_controller
        self.cmd_map_[limb_name].publish(JointTrajectory())

    def stop_trajectories(self):
        """
        Stop execution of all queued trajectories for all limbs
        """
        for name, cmd_pub in self.cmd_map_.items():

            cmd_pub.publish(JointTrajectory())


    def set_control_mode(self, mode, limb_name="left_arm"):
        """
        Switch to desired control mode
        @param mode: name of the desired control mode to be switched to
        options: ("cartesian_impedance", "joint_impedance","position")
        @param limb_name: name of the limb
        options: ("left_arm", "right_arm")
        """
        assert mode in ["cartesian_impedance", "joint_impedance","position"]
        
        

        if self._commander is not None:
            self._commander.stop()
            self._commander = None

        if mode == "cartesian_impedance":
            self._commander = CartesianImpedanceCommander(ns=limb_name)
            self._commander.activate()
        elif mode == "joint_impedance":
            self._commander = JointImpedanceCommander(ns=limb_name)
            self._commander.activate()

            self._commander.send_damping([25,25,25,25,10,0.01,0.001])#[0.1,0.1,0.1,0.1,0.1,0.1,0.1] #[25,25,25,25,10,0.01,0.001]
            self._commander.send_stiffness([250,250,200,100,60,50,10]) #[800,800,800,800,300,300,500]#[250,250,200,100,60,50,10]


        self._mode = mode

        rospy.sleep(6.0) # Give some time for the controller to settle in
        

    def exit_control_mode(self):

        if self._commander is not None:
            self._commander.stop()
            self._commander = None

            rospy.sleep(5.0) # Give some time for the controller to stop


    
    def cmd_joint_angles(self, angles, velocities=None, accelerations=None, execute=True):

        assert self._commander is not None and isinstance(self._commander,JointImpedanceCommander) \
            and self._commander.is_active()


        cmd = self._commander.compute_command(angles)

        
        if execute:
            self._commander.send_command(cmd)


        return cmd
        
    
    def cmd_cartesian_ee(self, ee_goal, execute=True):

        assert self._commander is not None and isinstance(self._commander,CartesianImpedanceCommander) \
            and self._commander.is_active()


        cmd = self._commander.compute_command(ee_goal)


        # For simulation only (BE CAREFUL)
        # cmd.k_FRI.x = cmd.k_FRI.y = cmd.k_FRI.z = 8000
        # cmd.k_FRI.rx = cmd.k_FRI.ry = cmd.k_FRI.rz = 2000

        cmd.k_FRI.x = cmd.k_FRI.y = cmd.k_FRI.z = 800
        cmd.k_FRI.rx = cmd.k_FRI.ry = cmd.k_FRI.rz = 50
        cmd.d_FRI.x = cmd.d_FRI.y = cmd.d_FRI.z = 0.65
        cmd.d_FRI.rx = cmd.d_FRI.ry = cmd.d_FRI.rz = 0.65

        if execute:
            self._commander.send_command(cmd)


    def set_joint_impedance(self, impedance):
        pass
    
    def set_joint_damping(self, damping):
        pass

    def set_cart_impedance(self, impedamce):
        pass

    def set_cart_damping(self, damping):
        pass
        

    #def end_effector(self)