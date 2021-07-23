#!/usr/bin/env python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

def pose():
    return np.array([-1.526626678081479, 1.5529365343163493, 1.6346674450878524, -1.769703802597152, -2.082327110779553, 2.095799026808471, 1.875109418979427])


def main(argv):

    rospy.init_node("message_poses", argv)

    pub = rospy.Publisher('desired_pose', Float64MultiArray, queue_size=10)

    pub.publish()

    while not rospy.is_shutdown():
        q_ik = ik.get_inverse_kinematics()
        data_to_send = Float64MultiArray()
        data_to_send.data = pose
        pub.publish(data_to_send)
        rospy.wait_for_service("TrajectoryExecution")


if __name__ == '__main__':
    main(sys.argv)