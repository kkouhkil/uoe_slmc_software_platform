#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

class RemapNode(object):
    """docstring for RemapNode"""

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
        self.msg_out.velocity = []
        self.msg_out.effort = []
        # Create JointState publisher
        self.pub = rospy.Publisher("/joint_states_output", JointState, queue_size=10)
        # Create JointState subscriber
        rospy.Subscriber("/joint_states_input", JointState, self.filter_joint_states_cb)

    def filter_joint_states_cb(self, msg_in):
        self.msg_out.header = msg_in.header
        # fill in joint state positions, velocities and efforts
        self.msg_out.position = []
        self.msg_out.velocity = []
        self.msg_out.effort = []
        for joint_name in self.msg_out.name:
            self.msg_out.position.append(msg_in.position[msg_in.name.index(joint_name)])
            self.msg_out.velocity.append(msg_in.velocity[msg_in.name.index(joint_name)])
            self.msg_out.effort.append(msg_in.effort[msg_in.name.index(joint_name)])
        # publish message
        self.pub.publish(self.msg_out)

if __name__ == '__main__':
    # initialize node
    rospy.init_node('remap_jointstate2jointstate', anonymous=True)
    # initialize class
    remap_node = RemapNode(rospy.get_name())
    # execute node
    rospy.spin()