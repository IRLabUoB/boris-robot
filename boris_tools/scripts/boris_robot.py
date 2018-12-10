#!/usr/bin/env python
from copy import deepcopy

import rospy
from sensor_msgs.msg import JointState

class BorisRobot(object):
    

    def __init__(self, name='boris'):

        self.name = name

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

        joint_state_topic = '/joint_states'
        self._joint_state_sub = rospy.Subscriber(
            joint_state_topic,
            JointState,
            self._on_joint_states,
            queue_size=1,
            tcp_nodelay=True)




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


    def joint_names(self):
        """
        Return the names of the joints for the specified limb.
        @rtype: [str]
        @return: ordered list of joint names from proximal to distal
        (i.e. shoulder to wrist).
        """
        return self._joint_names