#!/usr/bin/env python

import rospy
import numpy as np
import sys
from std_msgs.msg import Float64MultiArray
from z_trajectory_execution.srv import *
from z_trajectory_planning.srv import *
from trajectory_planning_calculation import getTargetStart
from trajectory_planning_calculation import ikLeastSquaresTest


def handle_next_pose(req):

    project_step = rospy.get_param("~project_step", "/default_project_step")

    if req.execution_success:

        if project_step == "CameraCalibration":
            camera_calibration_poses = np.loadtxt(filename_camera_calibration, delimiter=",")

        elif project_step == "HandEyeCalibration":
            camera_calibration_poses = np.loadtxt(filename_hand_eye_calibration, delimiter=",")

        elif project_step == "PointcloudRegistration":
            camera_calibration_poses = np.loadtxt(filename_pointcloud_registration, delimiter=",")

        else:
            # ________________CHANGE THIS____________________

            # camera_calibration_poses = np.array([[-1.808517652762068, 0.2444162242392147, 2.4911387847649897, -2.3705731577626765, -0.6074996608680785, 1.4251448823719681, 1.882035193794189]])
            rospy.logwarn("test")
            #___________Include Points from Vision Team!!!______________________
            p1 = np.array([0.25905142, 0.17681946, 0.16154686])
            p2 = np.array([0.25955791, 0.17693314, 0.16173824])
            p3 = np.array([0.25877628, 0.17553791, 0.16141655])
            p_target = np.array([0.28078221, 0.20254061, 0.17872114])
            # experimentally selected points:
            #p1 = np.array([0.2699134575743903, 0.014305403565485125, 0.18446952876503186])
            #p2 = np.array([0.34362474025668627, -0.05409762571615914, 0.20934292215990807])
            #p3 = np.array([0.40904724312988605, 0.024734139271587574, 0.1956804942851889])
            #p_target = np.array([0.31587545, -0.05159255, 0.12615232])
            T_goal = getTargetStart(p1, p2, p3, p_target)
            Q_0 = np.array([-1.02078, 0.306325, 2.41319, -1.99237, -1.65119, 1.47099, 0.301082])
            Q_goal = ikLeastSquaresTest(Q_0, T_goal)
            return NextPoseResponse(Q_goal)


        # rospy.logwarn(camera_calibration_poses.shape)

        curr_next_pose = camera_calibration_poses[req.n_pose, :]

        return NextPoseResponse(curr_next_pose)


def next_pose_server(argv):

    rospy.init_node("next_pose", argv)

    s = rospy.Service("next_pose", NextPose, handle_next_pose)

    rospy.spin()


if __name__ == '__main__':

    # filename = u"/home/rnm_grp3/catkin_ws/src/franka_ros/z_trajectory_planning/txt_files/calibration_poses_v2"
    filename_camera_calibration = u"/home/rnm_grp3/catkin_ws/src/franka_ros/z_trajectory_planning/txt_files/calibration_poses_v2"

    filename_hand_eye_calibration = u"/home/rnm_grp3/catkin_ws/src/franka_ros/z_trajectory_planning/txt_files/calibration_hand_eye"

    filename_pointcloud_registration = u"/home/rnm_grp3/catkin_ws/src/franka_ros/z_trajectory_planning/txt_files/pointcloud_poses"

    next_pose_server(sys.argv)
