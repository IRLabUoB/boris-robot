#!/usr/bin/env python

from aml_calib.hand_eye_calib import HandEyeCalib
from aml_calib.quaternion_utils import *
import numpy as np
import copy
import scipy.io as spio
from yml2npy import yml2transforms 



def rot2quat2(R):

    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = R[:3,:3].flat
    # Fill only lower half of symmetric matrix
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0              ],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
        ) / 3.0
    # Use Hermitian eigenvectors, values for speed
    vals, vecs = np.linalg.eigh(K)
    # Select largest eigenvector, reorder to x,y,z, w quaternion
    q = vecs[[0, 1, 2, 3], np.argmax(vals)]
    # Prefer quaternion with positive w
    # (q * -1 corresponds to same rotation as q)
    if q[0] < 0:
        q *= -1
    return q


def find_best_calib():
    p_gd = [0.090, 0.048, 0.058]
    q_gd = [0.490, 0.510, 0.492, 0.508]
    hand_transforms, camera_transforms = yml2transforms("TransformPairsInputTest02.yml", 26)


    best_error = None
    best_ridx = None

    best_transform = None
    best_p = None
    best_q = None
    for k in range(1):
        he_calib = HandEyeCalib()
        ridx = []
        for i in range(len(hand_transforms)):

            if np.random.rand() > 0.0:
                ridx.append(i)

        print "Selected indices: ", ridx
        for i in ridx:
            camera_pose = camera_transforms[i]
            gripper_pose = hand_transforms[i]
            #
            he_calib.add_measurement(gripper_pose, camera_pose)

        transform = he_calib.calibrate()

        p = transform[:3,3]
        q = rot2quat2(transform)

        dt, daxis_angle, drot_angle = posediff2(p_gd, q_gd, p, q)

        total_error = np.linalg.norm(dt) + np.abs(daxis_angle) + np.abs(drot_angle)

        if best_error is None or total_error < best_error:
            best_transform = transform
            best_p = p
            best_q = q
            best_error = total_error
            best_ridx = ridx
            print "[%d] Current best error: "%(k,), best_error

        

    print "Final error: ", best_error
    eu = quaternion2euler(best_q)
    print "PQ: \n", best_p
    print best_q
    print eu
    print "rosrun tf static_transform_publisher %f %f %f %f %f %f /left_arm_7_link /camera 10"%(best_p[0],best_p[1],best_p[2],eu[0],eu[1],eu[2])

    return {'transform': best_transform, 'p': best_p, 'q': best_q, 'eu': eu}

camera_base_p = [0.000, 0.022, 0.0]
camera_base_q = [-0.500, 0.500, -0.500, 0.500]
camera_base_transform = position_quaternion2matrix(camera_base_p, camera_base_q)

out = find_best_calib()

transform = np.matmul(out['transform'],np.linalg.inv(camera_base_transform))


p = transform[:3,3]
q = rot2quat2(transform)
eu = quaternion2euler(q) 
print "Final transform: "

print p
print eu

print "rosrun tf static_transform_publisher %f %f %f %f %f %f /left_arm_7_link /camera 10"%(p[0],p[1],p[2],eu[0],eu[1],eu[2])




# print "Found transform: \n", transform#np.linalg.inv(transform)
# print "Found inverse transform: \n", np.linalg.inv(transform)
# # print "Ground truth: \n", transform_gd

# p = transform[:3,3]
# q = rot2quat2(transform)

# p_inv = np.linalg.inv(transform)[:3,3]
# q_inv = rot2quat2(np.linalg.inv(transform))


# print "PQ: \n", p
# print q
# eu = quaternion2euler(q)
# print eu


# print "PQInv: \n", p_inv
# print q_inv
# eu_inv = quaternion2euler(q)
# print eu_inv


# print "TF: "
# print "rosrun tf static_transform_publisher %f %f %f %f %f %f /left_arm_7_link /camera 10"%(p[0],p[1],p[2],eu[0],eu[1],eu[2])
# print "TF_INV: "
# print "rosrun tf static_transform_publisher %f %f %f %f %f %f /left_arm_7_link /camera 10"%(p_inv[0],p[1],p_inv[2],eu_inv[0],eu_inv[1],eu_inv[2])

# dt, daxis_angle, drot_angle = posediff2(p_gd, q_gd, p, q)

# print "Error translation: \n", np.linalg.norm(dt)
# print "Error axis angle: \n", daxis_angle*180/np.pi
# print "Error rotation angle: \n", drot_angle*180/np.pi




