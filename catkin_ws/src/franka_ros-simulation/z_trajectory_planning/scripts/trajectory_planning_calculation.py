import numpy as np
from scipy.optimize import least_squares
import rospy

# _____________________________ variables __________________________________

# Current Position
# Q_init = np.array([-1.02078, 0.306325, 2.41319, -1.99237, -1.65119, 1.47099, 0.301082])

# End Position
# Q_end = np.array([-0.733962,-0.0525597,1.81821,-2.14711,-1.84831,1.83569,0.511367])  # just for testing, generally unknown
# T_end = getForwardKinematics(Q_end)

# Joint Space Limits of Panda Robot (from official Website)
JS_limits = np.array([
    [-2.89, -1.76, -2.89, -3.06, -2.89, -0.01, -2.89],
    [2.89, 1.76, 2.89, -0.07, 2.89, 3.7, 2.89]  # 0.8 -> 0.7 for testing
])


# _____________________________ functions _________________________________

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


def model(q):
    t = get_forward_kinematics(q)
    t = t[0:3, 0:4]
    t = t.transpose().reshape(1,12).ravel()
    return t


# Function for Least_Squares which is minimised
def fun(q, *y):
    return model(q) - y


def ik_least_square(q_start, t_final):
    y = t_final[0:-4]
    # y = y.transpose().reshape(1,12).ravel()
    ik = least_squares(fun, q_start, bounds=JS_limits, xtol=1e-3, ftol=1e-3, args=y, verbose=1)
    ik_res = ik.x
    return ik_res

def ikLeastSquaresTest(q_start, T_final):
    y = T_final[0:3, 0:4]
    y = y.transpose().reshape(1,12).ravel()
    ik = least_squares(fun, q_start, bounds=JS_limits, xtol=1e-3, ftol=1e-3, args=y, verbose=1)
    ik_res = ik.x
    return ik_res

def pos(a_0, a_1, a_2, a_3, a_4, a_5, t):
    return a_0 + a_1*t + a_2*t**2 + a_3*t**3 + a_4*t**4 + a_5*t**5


def vel(a_0, a_1, a_2, a_3, a_4, a_5, t):
    return a_1 + 2*a_2*t + 3*a_3*t**2 + 4*a_4*t**3 + 5*a_5*t**4


def acc(a_0, a_1, a_2, a_3, a_4, a_5, t):
    return 2*a_2 + 6*a_3*t + 12*a_4*t**2 + 20*a_5*t**3


def d_acc(a_0, a_1, a_2, a_3, a_4, a_5, t):
    return 6*a_3 + 24*a_4*t + 60*a_5*t**2


def get_a_0(q_0):
    return q_0


def get_a_1(v_0):
    return v_0


def get_a_2(ac_0):
    return ac_0/2


def get_a_3(q_0, v_0, ac_0, q_f, v_f, ac_f, t_f):
    a = (20*q_f - 20*q_0 - (8*v_f + 12*v_0)*t_f - (3*ac_0 - ac_f)*t_f**2)/(2*t_f**3)
    #  a = 20*q_f - 20*q_0 - (8*v_f + 12*v_0)*t_f - (3*ac_0 - 2*ac_f)*pow(t_f, 2)
    # b = 2*pow(t_f, 3)
    # return a/b
    return a


def get_a_4(q_0, v_0, ac_0, q_f, v_f, ac_f, t_f):
    a = (30*q_0 - 30*q_f + (14*v_f + 16*v_0)*t_f + (3*ac_0 - 2*ac_f)*t_f**2)/(2*t_f**4)
    # a = 30*q_0 - 30*q_f + (14*v_f + 16*v_0)*t_f + (3*ac_0 - 2*ac_f)*pow(t_f, 2)
    # b = 2*pow(t_f, 4)
    # return a/b
    return a


def get_a_5(q_0, v_0, ac_0, q_f, v_f, ac_f, t_f):
    a = (12*q_f - 12*q_0 - (6*v_f + 6*v_0)*t_f - (ac_0 - ac_f)*t_f**2)/(2*t_f**5)
    # a = 12*q_f - 12*q_0 - (6*v_f + 6*v_0)*t_f - (ac_0 - ac_f)*pow(t_f, 2)
    # b = 2*pow(t_f, 5)
    # return a/b
    return a

def get_quintic_trajectory(q_0, q_f, v_0, v_f, ac_0, ac_f):

    #vel_panda_max = np.array([2.17, 2.17, 2.17, 2.17, 2.6, 2.6, 2.6]) # rad/s -----------ORIGINAL VEL CONST
    vel_panda_max = np.array([1.5, 1.5, 1.5, 1.5, 2.0, 2.0, 2.0]) # rad/s -----------REDUCED VEL CONST

    #acc_panda_max = np.array([15, 7.5, 10, 12.5, 15, 20, 20]) # rad/s^2 ---------------ORIGINAL ACC CONST
    acc_panda_max = np.array([12, 6, 8, 10, 12, 15, 15]) # rad/s^2 ---------------REDUCED ACC CONST
    #acc_panda_max = acc_panda_max*0.2e-3

    # d_acc_panda_max = np.array([7500, 3750, 5000, 6250, 7500, 10000, 10000]) # rad/s^3 -------------ORIGINAL JERK CONST
    d_acc_panda_max = np.array([6000, 2500, 4000, 5000, 6000, 9000, 9000]) # rad/s^3 -------------REDUCED JERK CONST
    #d_acc_panda_max = d_acc_panda_max*0.2e-4

    t_f = 1000

    const_met=False

    while const_met == False:
        print(t_f)

        # Get current time variables
        t = np.arange(t_f)
        t_f_s = t_f/1000
        t_s = np.true_divide(t, 1000)

        # Inner loop flag
        t_inc = False

        # Initialize resolving path
        path_res = np.zeros((7, t_f))

        for i in range(0,7):
            a_0 = get_a_0(q_0[i])
            a_1 = get_a_1(v_0[i])
            a_2 = get_a_2(ac_0[i])
            a_3 = get_a_3(q_0[i], v_0[i], ac_0[i], q_f[i], v_f[i], ac_f[i], t_f_s)
            a_4 = get_a_4(q_0[i], v_0[i], ac_0[i], q_f[i], v_f[i], ac_f[i], t_f_s)
            a_5 = get_a_5(q_0[i], v_0[i], ac_0[i], q_f[i], v_f[i], ac_f[i], t_f_s)

            # Get path
            path_i = pos(a_0, a_1, a_2, a_3, a_4, a_5, t_s)
            # Get and check velocity
            vel_i = vel(a_0, a_1, a_2, a_3, a_4, a_5, t_s)
            if np.max(np.abs(vel_i)) > vel_panda_max[i]:
                t_inc = True
                break

            # Get and check acceleration
            acc_i = acc(a_0, a_1, a_2, a_3, a_4, a_5, t_s)
            if np.max(np.abs(acc_i)) > acc_panda_max[i]:
                t_inc = True
                break

            # Get and check jerk
            d_acc_i = d_acc(a_0, a_1, a_2, a_3, a_4, a_5, t_s)
            if np.max(np.abs(d_acc_i)) > d_acc_panda_max[i]:
                t_inc = True
                break

            print(path_i.shape)
            print("test")
            #plt.plot(path_i)
            #plt.plot(vel_i)
            #plt.plot(acc_i)
            #plt.plot(d_acc_i)
            #plt.legend(["path", "vel", "acc", "d_acc"])
            #plt.show()

            path_res[i, :] = pos(a_0, a_1, a_2, a_3, a_4, a_5, t_s)

        if t_inc == True:
            t_f = t_f + 1000
        else:
            last_pos = q_f.reshape(7, 1)
            path_res = np.append(path_res, last_pos, axis=1)
            const_met = True

    return path_res


def s1_straight_path(d_z, n_i, n):
    if n == 0:
        return 0
    else:
        a = 2*(d_z)/(n**2)
        b = 0
        c = 0
        return a*n_i**2+b*n_i+c

def s2_straight_path(d_z, n_i, n):
    d = -(2*d_z)/(n**2)
    e = 4*d_z/n
    f = -d_z
    return d*n_i**2+e*n_i+f


def get_delta_z(delta_z, n_i, n):
    if n_i < int(n/2):
        return s1_straight_path(delta_z, n_i, n)
    else:
        return s2_straight_path(delta_z, n_i, n)

def get_straight_line_ee(q_init, delta_z):

    # q_curr = np.array(q_curr)
    n = int(60000*delta_z)  # number of points
    t_init = get_forward_kinematics(q_init)  # T_init
    q_curr = q_init
    joint_angles = np.zeros((7, n+1))

    t_8ee = np.identity(4)

    for i in range(0,n+1):

        t_8ee[2,3] = get_delta_z(delta_z, i, n)
        rospy.logwarn(t_8ee[2,3])

        t_next = np.matmul(t_init, t_8ee)

        # rospy.logwarn(t_next)
        # t_ik = t_next.reshape(16,1).ravel()
        # rospy.logwarn(t_ik)
        # rospy.logwarn(i)

        q_curr = np.array(q_curr)
        joint_angles[:, i] = ikLeastSquaresTest(q_curr, t_next)
        q_curr = joint_angles[:, i]
        # rospy.sleep(6)
    rospy.logwarn(joint_angles.shape)

    return joint_angles


def getTargetStart(p1, p2, p3, p_target):

    # calculate centroid of triangle
    m_x = np.round((p1[0] + p2[0] + p3[0]) / 3, 2)
    m_y = np.round((p1[1] + p2[1] + p3[1]) / 3, 2)
    m_z = np.round((p1[2] + p2[2] + p3[2]) / 3, 2)
    M = np.array([m_x, m_y, m_z])
    rospy.logwarn("This is M")
    rospy.logwarn(M)

    # vector from centroid to target
    if M[1] > p_target[1]:
        z_vec = p_target - M
    else:
        z_vec = M - p_target

    #z_vec = M - p_target
    z_vec = z_vec * 4
    z_vec_norm = z_vec / (np.sqrt(np.sum(z_vec**2)))
    # start point
    P_start = p_target - z_vec

    # coordinate system at start point
    z_R = z_vec_norm
    x_3 = (-z_R[0] - z_R[1]) / z_R[2]
    x_R = np.array([1, 1, x_3])
    x_R = x_R / (np.sqrt(np.sum(x_R**2)))
    y_R = np.cross(z_R, x_R)

    # rotation matrix built up out of new coordinate system vectors
    R = np.array([x_R, y_R, z_R])
    R = R.T

    # transformation matrix (desired pose)
    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = P_start

    return T


def getTargetDist(p_target, q_start):
    needle_length = 0.16
    T_start = get_forward_kinematics(q_start)
    p_start = T_start[0:3, 3]
    print(p_start, '\n')
    dist = np.linalg.norm(p_target - p_start) - needle_length
    return abs(dist)
