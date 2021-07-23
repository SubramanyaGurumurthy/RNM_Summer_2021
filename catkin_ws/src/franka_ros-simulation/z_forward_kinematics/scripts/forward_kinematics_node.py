#!/usr/bin/env python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from forward_kinematics_calculation import get_forward_kinematics # !--Change this--!


class ForwardKinematics(object):

    def __init__(self, command_topic):

        if "sim" in command_topic:
            msg = rospy.wait_for_message("/joint_states", JointState, rospy.Duration(10))
        else:
            msg = rospy.wait_for_message("/franka_state_controller/joint_states_desired", JointState, rospy.Duration(10))

        self.position = msg.position

    def get_forward_kinematics(self):
        q_init = self.position
        q_init = np.array(q_init).astype(np.float64)
        fk = get_forward_kinematics(q_init)
        # rospy.logwarn(fk)
        return fk


def main(argv):

    rospy.init_node("forward_kinematics", argv)

    command_topic = rospy.get_param("~command_topic", "/default_command_topic")

    # rospy.logwarn("Using topic " + str(command_topic))

    pub = rospy.Publisher('fk', Float64MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        fk = ForwardKinematics(command_topic)
        tf_08 = fk.get_forward_kinematics()
        data_to_send = Float64MultiArray()
        tf_08 = tf_08.flatten()
        data_to_send.data = tf_08
        pub.publish(data_to_send)
        rospy.sleep(0.1)


if __name__ == '__main__':
    main(sys.argv)
