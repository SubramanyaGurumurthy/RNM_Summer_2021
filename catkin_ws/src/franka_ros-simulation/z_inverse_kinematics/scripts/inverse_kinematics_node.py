#!/usr/bin/env python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from inverse_kinematics_calculation import ik_least_square # !--Change this--!


class InverseKinematics(object):
    def __init__(self, command_topic):

        if "sim" in command_topic:
            msg = rospy.wait_for_message("/joint_states", JointState, rospy.Duration(10))
        else:
            msg = rospy.wait_for_message("/franka_state_controller/joint_states_desired", JointState, rospy.Duration(10))

        self.position = msg.position

        # testing purposes
        msg_2 = rospy.wait_for_message("/fk", Float64MultiArray, rospy.Duration(10))

        self.tf = msg_2.data

    def get_inverse_kinematics(self):
        tf_08 = self.tf
        q_init = self.position
        tf_08 = np.array(tf_08).astype(np.float64)
        ik = ik_least_square(q_init, tf_08)
        return ik


def main(argv):

    rospy.init_node("forward_kinematics", argv)

    command_topic = rospy.get_param("~command_topic", "/default_command_topic")

    # rospy.logwarn("Using topic " + str(command_topic))

    pub = rospy.Publisher('ik', Float64MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        ik = InverseKinematics(command_topic)
        q_ik = ik.get_inverse_kinematics()
        data_to_send = Float64MultiArray()
        data_to_send.data = q_ik
        pub.publish(data_to_send)
        rospy.sleep(0.1)


if __name__ == '__main__':
    main(sys.argv)
