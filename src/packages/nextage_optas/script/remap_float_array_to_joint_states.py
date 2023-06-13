#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class RemapNode(object):
    """docstring for FilterJointState"""

    def __init__(self, name) -> None:
        # Initialization message
        self.name = name
        rospy.loginfo("%s: Initialized.", self.name)
        # Create joints msg
        self.msg_out = JointState()
        self.msg_out.name = [
            "CHEST_JOINT0",
            "HEAD_JOINT0",
            "HEAD_JOINT1",
            "LARM_JOINT0",
            "LARM_JOINT1",
            "LARM_JOINT2",
            "LARM_JOINT3",
            "LARM_JOINT4",
            "LARM_JOINT5",
            "RARM_JOINT0",
            "RARM_JOINT1",
            "RARM_JOINT2",
            "RARM_JOINT3",
            "RARM_JOINT4",
            "RARM_JOINT5",
        ]
        self.msg_out.position = []
        # Create JointState publisher
        self.pub = rospy.Publisher("/joint_states_output", JointState, queue_size=10)
        # Create JointState subscriber
        rospy.Subscriber("/float_array_input", Float64MultiArray, self.remap_cb)

    def remap_cb(self, msg_in):
        self.msg_out.position = list(msg_in.data)
        self.pub.publish(self.msg_out)


if __name__ == '__main__':
    # initialize node
    rospy.init_node('remap_floatarray2jointstate', anonymous=True)
    # initialize class
    remap_node = RemapNode(rospy.get_name())
    # execute node
    rospy.spin()