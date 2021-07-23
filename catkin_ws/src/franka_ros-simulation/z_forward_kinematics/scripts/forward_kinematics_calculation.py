#!/usr/bin/env python
import numpy as np


def get_dh_params(q):
    return np.array([[0, 0.333, 0, q[0]],  # a, d, alpha, phi
                     [0, 0, -np.pi/2, q[1]],
                     [0, 0.316, np.pi/2, q[2]],
                     [0.0825, 0, np.pi/2, q[3]],
                     [-0.0825, 0.384, -np.pi/2, q[4]],
                     [0, 0, np.pi/2, q[5]],
                     [0.088, 0, np.pi/2, q[6]],
                     [0, 0.107, 0, 0]
                     ], dtype=float)


def transform_matrix(i, dh):
    tm = np.array([[np.cos(dh[i, 3]), -np.sin(dh[i, 3]), 0, dh[i, 0]],
                  [np.sin(dh[i, 3])*np.cos(dh[i, 2]), np.cos(dh[i, 3])*np.cos(dh[i, 2]),
                   -np.sin(dh[i, 2]), -dh[i, 1]*np.sin(dh[i, 2])],
                  [np.sin(dh[i, 3])*np.sin(dh[i, 2]), np.cos(dh[i, 3])*np.sin(dh[i, 2]),
                   np.cos(dh[i, 2]), dh[i, 1]*np.cos(dh[i, 2])],
                  [0, 0, 0, 1]
                   ], dtype=float)
    return tm


def get_forward_kinematics(q):
    dh = get_dh_params(q)
    tf_01 = transform_matrix(0, dh) # base to KOS 1
    tf_12 = transform_matrix(1, dh) # KOS 1 to KOS 2
    tf_23 = transform_matrix(2, dh) # KOS 2 to KOS 3
    tf_34 = transform_matrix(3, dh) # KOS 3 to KOS 4
    tf_45 = transform_matrix(4, dh)
    tf_56 = transform_matrix(5, dh)
    tf_67 = transform_matrix(6, dh)
    tf_78 = transform_matrix(7, dh)

    tf_03 = np.matmul(tf_01, np.matmul(tf_12, tf_23))
    tf_36 = np.matmul(tf_34, np.matmul(tf_45, tf_56))
    tf_68 = np.matmul(tf_67, tf_78)

    tf_08 = np.matmul(tf_03, np.matmul(tf_36, tf_68))

    return tf_08
