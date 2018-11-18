#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

X, Y, Z, W = 0, 1, 2, 3

def quaternion_rotate_vec(q, v):
    px = q[Y] * v[Z] - v[Y] * q[Z] + q[W] * v[X]
    py = q[Z] * v[X] - v[Z] * q[X] + q[W] * v[Y]
    pz = q[X] * v[Y] - v[X] * q[Y] + q[W] * v[Z]

    p2x = px + px
    p2y = py + py
    p2z = pz + pz

    return np.array((v[X] + (q[Y] * p2z - p2y * q[Z]),
                     v[Y] + (q[Z] * p2x - p2z * q[X]),
                     v[Z] + (q[X] * p2y - p2x * q[Y])), dtype=np.float64)

def quaternion_inv_rotate_vec(q, v):
    px = q[Y] * v[Z] - v[Y] * q[Z] - q[W] * v[X]
    py = q[Z] * v[X] - v[Z] * q[X] - q[W] * v[Y]
    pz = q[X] * v[Y] - v[X] * q[Y] - q[W] * v[Z]

    p2x = px + px
    p2y = py + py
    p2z = pz + pz

    return np.array((v[X] + (q[Y] * p2z - p2y * q[Z]),
                     v[Y] + (q[Z] * p2x - p2z * q[X]),
                     v[Z] + (q[X] * p2y - p2x * q[Y])), dtype=np.float64)

