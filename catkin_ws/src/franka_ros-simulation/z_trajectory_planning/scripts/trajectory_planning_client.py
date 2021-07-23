#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Bool
# from xmlrpc.client import Boolean
from trajectory_planning_calculation import get_quintic_trajectory
from trajectory_planning_calculation import get_straight_line_ee # !--Change this--!
from trajectory_planning_calculation import getTargetDist
from z_trajectory_execution.srv import *
from z_trajectory_planning.srv import *


class TrajectoryPlanning(object):

    # Test Position
    # q_test = np.array([-2.2388072014552063, 1.0582324061880892, 1.7444477332795054, -2.3173156585110037,
    # -1.7118003667998898, 2.9961753818194072, 1.7551842792995485])

    v0 = np.zeros(7)
    v1 = np.zeros(7)
    a0 = np.zeros(7)
    a1 = np.zeros(7)

    delta_z = 0.2

    def __init__(self, command_topic):

        rospy.wait_for_service('trajectory_execution')

        if "sim" in command_topic:
            msg = rospy.wait_for_message("/joint_states", JointState, rospy.Duration(10))
        else:
            msg = rospy.wait_for_message("/franka_state_controller/joint_states_desired", JointState, rospy.Duration(10))

        self.position = msg.position

    def get_trajectory_planning(self, path_type, goal_pose):

        q_init = self.position

        # ____________________Next Goal Pose______________________________________

        q_goal = np.array(goal_pose)

        if path_type == "quintic":
            path_res = get_quintic_trajectory(q_init, q_goal, self.v0, self.v1, self.a0, self.a1)
        else:
            # __________Include target Point!!!_______________
            end_point = np.array([0.31587545, -0.05159255, 0.12615232])
            z_dist = getTargetDist( end_point, q_init)
            rospy.logwarn(z_dist)
            path_res = get_straight_line_ee(q_init, z_dist)

        return path_res


def trajectory_execution_client(path_res):
    rospy.wait_for_service("trajectory_execution")

    try:

        trajectory_execution = rospy.ServiceProxy("trajectory_execution", TrajectoryExecution)
        resp1 = trajectory_execution(path_res[0, :], path_res[1, :], path_res[2, :], path_res[3, :],
                                     path_res[4, :], path_res[5, :], path_res[6, :])

        return resp1.execution_success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def next_pose_client(execution_success, n_pose):
    rospy.wait_for_service("next_pose")

    try:

        next_pose = rospy.ServiceProxy("next_pose", NextPose)
        resp1_next_pose = next_pose(execution_success, n_pose)

        return resp1_next_pose.next_pose
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def main(argv):

    rospy.init_node("trajectory_planning", argv)

    command_topic = rospy.get_param("~command_topic", "/default_command_topic")

    path_type = rospy.get_param("~path_type", "/default_path_type")

    project_step = rospy.get_param("~project_step", "/default_project_step")

    pub = rospy.Publisher('/ready_for_image', Bool, queue_size=10)

    # First pose
    execution_success = True

    n_pose = 0

    while not rospy.is_shutdown():

        tp = TrajectoryPlanning(command_topic)

        if project_step == "TargetExecution":

            if n_pose == 0:
                resp1_next_pose = next_pose_client(execution_success, n_pose)

                rospy.logwarn(resp1_next_pose)
                rospy.logwarn("Iteration:    " + str(n_pose))

                path_res = tp.get_trajectory_planning(path_type, resp1_next_pose)
                rospy.logwarn(path_res)
                trajectory_execution_client(path_res)

                n_pose = n_pose + 1

            elif n_pose == 1:

                path_type = "straight"

                resp1_next_pose = np.zeros(7)

                path_res = tp.get_trajectory_planning(path_type, resp1_next_pose)
                trajectory_execution_client(path_res)

                n_pose = n_pose + 1

            else:
                rospy.logwarn("Success!!!")
                rospy.sleep(1000)

        else:

            resp1_next_pose = next_pose_client(execution_success, n_pose)

            rospy.logwarn(resp1_next_pose)
            rospy.logwarn("Iteration:    " + str(n_pose))

            path_res = tp.get_trajectory_planning(path_type, resp1_next_pose)
            trajectory_execution_client(path_res)

            pub.publish(True)

            #rospy.sleep(2)

            image_taken = True
            while image_taken:
                try:
                     image_taken = rospy.wait_for_message('/ready_for_image', Bool, timeout=1)
                     image_taken = image_taken.data
                except:
                    pass

            n_pose = n_pose + 1




if __name__ == '__main__':
    main(sys.argv)



