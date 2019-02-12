#!/usr/bin/python

import numpy as np
import PyKDL
from kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
import tf.transformations as tt
import rospy
from trac_ik_python.trac_ik import IK

def joint_list_to_kdl(q):
    if q is None:
        return None
    if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]
    q_kdl = PyKDL.JntArray(len(q))
    for i, q_i in enumerate(q):
        q_kdl[i] = q_i
    return q_kdl


class boris_kinematics(object):
    def __init__(self, root_link=None, tip_link="left_arm_7_link"):
        self._urdf = URDF.from_parameter_server(key='robot_description')
        self._kdl_tree = kdl_tree_from_urdf_model(self._urdf)

        self._base_link = self._urdf.get_root() if root_link is None else root_link
        self._tip_link = tip_link
        self._arm_chain = self._kdl_tree.getChain(self._base_link, self._tip_link)

        self._joint_names = self._urdf.get_chain(self._base_link, self._tip_link, links=False, fixed=False)
        self._num_jnts = len(self._joint_names)

        # KDL
        self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._arm_chain)
        self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._arm_chain, self._fk_p_kdl, self._ik_v_kdl)

        # trac_ik
        urdf_str = rospy.get_param('/robot_description')
        self.trac_ik_solver = IK(self._base_link, self._tip_link, urdf_string=urdf_str)

        self.joint_limits_lower = []
        self.joint_limits_upper = []
        self.joint_types = []

        for jnt_name in self._joint_names:
            jnt = self._urdf.joint_map[jnt_name]
            if jnt.limit is not None:
                self.joint_limits_lower.append(jnt.limit.lower)
                self.joint_limits_upper.append(jnt.limit.upper)
            else:
                self.joint_limits_lower.append(None)
                self.joint_limits_upper.append(None)
            self.joint_types.append(jnt.type)
        


        self._default_seed = [-0.3723750412464142, 0.02747996523976326, -0.7221865057945251, -1.69843590259552, 0.11076358705759048, 0.5450965166091919, 0.45358774065971375] # home_pos

        #lower_lim = np.where(np.isfinite(self.joint_limits_lower), self.joint_limits_lower, 0.)
        #upper_lim = np.where(np.isfinite(self.joint_limits_upper), self.joint_limits_upper, 0.)
        #seed = (lower_lim + upper_lim) / 2.0
        #self._default_seed = np.where(np.isnan(seed), [0.]*len(seed), seed) # mid of the joint limits

    def forward_kinematics(self, joint_values):
        kdl_array = joint_list_to_kdl(joint_values)

        end_frame = PyKDL.Frame()
        self._fk_p_kdl.JntToCart(kdl_array, end_frame)

        pos = end_frame.p
        quat = PyKDL.Rotation(end_frame.M).GetQuaternion()
        return np.array([pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]])

    def trac_ik_inverse_kinematics(self, pos, quat, seed=None):
        if seed is None:
            seed = self._default_seed
        result = self.trac_ik_solver.get_ik(seed,
                                            pos[0], pos[1], pos[2],
                                            quat[0], quat[1], quat[2], quat[3])
        # return np.array(result)
        return True if result is not None else False

    def kdl_inverse_kinematics(self, pos, quat, seed=None):
        pos = PyKDL.Vector(pos[0], pos[1], pos[2])
        rot = PyKDL.Rotation()
        rot = rot.Quaternion(quat[0], quat[1], quat[2], quat[3])
        goal_pose = PyKDL.Frame(rot, pos)

        seed_array = joint_list_to_kdl(self._default_seed)
        if seed != None:
            seed_array = joint_list_to_kdl(seed)

        result_angles = PyKDL.JntArray(self._num_jnts)
        if self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = np.array(list(result_angles))
            # return result
            return True
        else:
            # return None
            return False

    def kdl_inverse_kinematics_jl(self, pos, quat, seed=None, min_joints=None, max_joints=None):
        pos = PyKDL.Vector(pos[0], pos[1], pos[2])
        rot = PyKDL.Rotation()
        rot = rot.Quaternion(quat[0], quat[1], quat[2], quat[3])
        goal_pose = PyKDL.Frame(rot, pos)

        if min_joints is None:
            min_joints = self.joint_limits_lower
        if max_joints is None:
            max_joints = self.joint_limits_upper
        mins_kdl = joint_list_to_kdl(min_joints)
        maxs_kdl = joint_list_to_kdl(max_joints)

        ik_jl_p_kdl = PyKDL.ChainIkSolverPos_NR_JL(self._arm_chain, mins_kdl, maxs_kdl,
                                                   self._fk_p_kdl, self._ik_v_kdl)

        seed_array = joint_list_to_kdl(self._default_seed)
        if seed != None:
            seed_array = joint_list_to_kdl(seed)

        result_angles = PyKDL.JntArray(self._num_jnts)
        if ik_jl_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = np.array(list(result_angles))
            # return result
            return True
        else:
            # return None
            return False

if __name__ == '__main__':

    kin = boris_kinematics()
    joint_pos = [-0.3723750412464142, 0.02747996523976326, -0.7221865057945251, -1.69843590259552, 0.11076358705759048, 0.5450965166091919, 0.45358774065971375] # home pos
    pose = kin.forward_kinematics(joint_pos) # pos & quat

    print ''
    print '*** IK solution test ***'
    seed = None
    for i in range(5):
        pose[0] += 0.01

        has_solution = kin.kdl_inverse_kinematics(pose[0:3], pose[3:7], seed)
        print 'kdl w/o jl:', has_solution

        has_solution = kin.kdl_inverse_kinematics_jl(pose[0:3], pose[3:7], seed)
        print 'kdl w/  jl:', has_solution

        has_solution = kin.trac_ik_inverse_kinematics(pose[0:3], pose[3:7], seed)
        print 'trac ik   :', has_solution
        print ''
