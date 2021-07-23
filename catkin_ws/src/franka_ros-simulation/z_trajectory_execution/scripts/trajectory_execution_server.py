#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from z_trajectory_execution.srv import *
from sensor_msgs.msg import JointState


class RobotArm:
    def __init__(self, command_topic):

        if "sim" in command_topic:
            msg = rospy.wait_for_message("/joint_states", JointState, rospy.Duration(10))
        else:
            msg = rospy.wait_for_message("/franka_state_controller/joint_states_desired", JointState, rospy.Duration(10))

        self.initial_position = msg.position

        # Setup the publisher
        self.publisher = rospy.Publisher(command_topic, Float64MultiArray, queue_size=1)
        self.counter = 0

    def send_step_command(self, pose):

        rate = rospy.Rate(1000)
        msg = Float64MultiArray()
        msg.data = pose
        self.publisher.publish(msg)
        rate.sleep()


def handle_trajectory_execution(req):

    joint1 = np.array(req.joint1)
    joint2 = np.array(req.joint2)
    joint3 = np.array(req.joint3)
    joint4 = np.array(req.joint4)
    joint5 = np.array(req.joint5)
    joint6 = np.array(req.joint6)
    joint7 = np.array(req.joint7)

    rospy.logwarn(joint1.shape[0])

    command_topic = rospy.get_param("~command_topic", "/default_command_topic")
    robot_arm = RobotArm(command_topic)

    for i in range(0, joint1.shape[0]):

        msg = np.array([joint1[i], joint2[i], joint3[i], joint4[i], joint5[i], joint6[i], joint7[i]])

        robot_arm.send_step_command(msg)

    return TrajectoryExecutionResponse(True)


def trajectory_execution_server(argv):

    rospy.init_node("trajectory_execution", argv)

    s = rospy.Service("trajectory_execution", TrajectoryExecution, handle_trajectory_execution)

    rospy.spin()


if __name__ == '__main__':
    trajectory_execution_server(sys.argv)
