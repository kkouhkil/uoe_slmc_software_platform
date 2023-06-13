#!/usr/bin/env python3
import rospy
import actionlib
from sensor_msgs.msg import Joy
from pushing_msgs.msg import TriggerCmdAction, TriggerCmdGoal
from pushing_msgs.msg import CmdDualPoseAction, CmdDualPoseGoal
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal

class TeleOp3DClient(object):
    """docstring for TeleOp3DClient."""

    def __init__(self, name) -> None:
        # initialization message
        self._name = name
        rospy.loginfo("%s: Initialized action client class.", self._name)
        #################################################################################
        # get parameters
        self._freq = rospy.get_param('~freq', 100)
        self._duration = rospy.get_param('~duration', 2.0)
        self._target_pos_x_R = rospy.get_param('~target_position_x_R', 0.3)
        self._target_pos_y_R = rospy.get_param('~target_position_y_R', -0.2)
        self._target_pos_z_R = rospy.get_param('~target_position_z_R', 0.83)
        self._safety_pos_x_R = rospy.get_param('~safety_position_x_R', 0.3)
        self._safety_pos_y_R = rospy.get_param('~safety_position_y_R', -0.1)
        self._safety_pos_z_R = rospy.get_param('~safety_position_z_R', 0.95)
        self._target_pos_x_L = rospy.get_param('~target_position_x_L', 0.3)
        self._target_pos_y_L = rospy.get_param('~target_position_y_L', 0.2)
        self._target_pos_z_L = rospy.get_param('~target_position_z_L', 0.90)
        self._safety_pos_x_L = rospy.get_param('~safety_position_x_L', 0.3)
        self._safety_pos_y_L = rospy.get_param('~safety_position_y_L', 0.3)
        self._safety_pos_z_L = rospy.get_param('~safety_position_z_L', 0.95)
        # griper parameters
        self.gripper_position_open = rospy.get_param('~gripper_position_open', 0.08)
        self.gripper_position_close = rospy.get_param('~gripper_position_close', 0.0)
        self.gripper_speed = rospy.get_param('~gripper_speed', 0.01)
        self.gripper_force = rospy.get_param('~gripper_force', 5)
        # declare joystick substriber
        self._joy_sub = rospy.Subscriber(
            "/spacenav/joy",
            Joy,
            self.read_joy_status_cb
        )
        # declare timer for the State Machine loop
        dur = rospy.Duration(1.0/self._freq)
        self._timer_arm = rospy.Timer(dur, self.timer_arm_cb)
        self._timer_gripper = rospy.Timer(dur, self.timer_gripper_cb)
        #################################################################################
        # initialized variables
        self.joyButtonLeft = False
        self.joyButtonRight = False
        self.reachedArmTargetGoal = False
        self.reachedArmSafetyGoal = False
        self.closedGripper = False
        self.openedGripper = False
        # initialize SM states
        self.stateArmInactive = True
        self.stateTeleOp = False
        self.stateSendArm2Target = False
        self.stateSendArm2Safety = False
        # gripper 
        self.stateGripperInactive = True
        self.stateCatching = False
        self.stateOpenGripper = False
        self.stateCloseGripper = False
        # start innactive state
        self.inactive_arm_cb()
        self.inactive_gripper_cb()
        #################################################################################
        # create action client to teleop robot
        self.action_client_teleop = actionlib.SimpleActionClient(rospy.get_namespace() + "cmd_twist", TriggerCmdAction)
        # create goal
        self.goal_teleop = TriggerCmdGoal(pos_z=self._target_pos_z_R)
        #################################################################################
        # create action client for commanding pose
        self.action_client_cmd_pose = actionlib.SimpleActionClient(rospy.get_namespace() + "cmd_pose", CmdDualPoseAction)
        # create goal
        self.goal_cmd_pose = CmdDualPoseGoal()
        self.goal_cmd_pose.poseR.position.x = self._safety_pos_x_R
        self.goal_cmd_pose.poseR.position.y = self._safety_pos_y_R
        self.goal_cmd_pose.poseR.position.z = self._safety_pos_z_R
        self.goal_cmd_pose.poseR.orientation.x = 1
        self.goal_cmd_pose.poseR.orientation.y = 0
        self.goal_cmd_pose.poseR.orientation.z = 0
        self.goal_cmd_pose.poseR.orientation.w = 0
        self.goal_cmd_pose.poseL.position.x = self._safety_pos_x_L
        self.goal_cmd_pose.poseL.position.y = self._safety_pos_y_L
        self.goal_cmd_pose.poseL.position.z = self._safety_pos_z_L
        self.goal_cmd_pose.poseL.orientation.x = 1
        self.goal_cmd_pose.poseL.orientation.y = 0
        self.goal_cmd_pose.poseL.orientation.z = 0
        self.goal_cmd_pose.poseL.orientation.w = 0
        self.goal_cmd_pose.duration = self._duration
        #################################################################################
        # creat action client for gripper
        action_name = rospy.get_param('~action_name', '/command_robotiq_action')
        self.action_client_robotiq = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
        # create goal for gripper
        self.goal_cmd_robotiq = CommandRobotiqGripperGoal()
        self.goal_cmd_robotiq.emergency_release = False
        self.goal_cmd_robotiq.stop = False
        self.goal_cmd_robotiq.position = 0.0
        self.goal_cmd_robotiq.speed = self.gripper_speed
        self.goal_cmd_robotiq.force = self.gripper_force

    def read_joy_status_cb(self, msg):
        options = [False, True]
        self.joyButtonLeft = options[msg.buttons[0]]
        self.joyButtonRight = options[msg.buttons[1]]

    def timer_arm_cb(self, event):
        """ Manage State Machine logic for arm motion """
        if self.stateArmInactive and self.joyButtonLeft:
            self.stateArmInactive = False
            self.stateSendArm2Target = True
            self.reachedArmTargetGoal = self.cmd_arm_cb(
                self._target_pos_x_R, self._target_pos_y_R, self._target_pos_z_R,
                self._target_pos_x_L, self._target_pos_y_L, self._target_pos_z_L,
                'Workspace')
        elif self.stateSendArm2Target and self.reachedArmTargetGoal:
            self.stateSendArm2Target = False
            self.reachedArmTargetGoal = False
            self.stateTeleOp = True
            self.teleop_cb()
        elif self.stateTeleOp and self.joyButtonLeft:
            self.stateSendArm2Safety = True
            self.stateTeleOp = False
            self.action_client_teleop.cancel_goal()
            self.reachedArmSafetyGoal = self.cmd_arm_cb(
                self._safety_pos_x_R, self._safety_pos_y_R, self._safety_pos_z_R,
                self._safety_pos_x_L, self._safety_pos_y_L, self._safety_pos_z_L,
                'Safety')
        elif self.stateSendArm2Safety and self.reachedArmSafetyGoal:
            self.stateSendArm2Safety = False
            self.reachedArmSafetyGoal = False
            self.stateArmInactive = True
            self.inactive_arm_cb()
        else:
            pass

    def timer_gripper_cb(self, event):
        """ Manage State Machine logic for arm motion """
        if self.stateGripperInactive and self.joyButtonRight:
            self.stateGripperInactive = False
            self.stateOpenGripper = True
            self.openedGripper = self.cmd_robotiq_cb(self.gripper_position_open)
        elif self.stateOpenGripper and self.openedGripper:
            self.stateOpenGripper = False
            self.openedGripper = False
            self.stateCatching = True
        elif self.stateCatching and self.joyButtonRight:
            self.stateCatching = False
            self.stateCloseGripper = True
            self.closedGripper = self.cmd_robotiq_cb(self.gripper_position_close)
        elif self.stateCloseGripper and self.closedGripper:
            self.stateCloseGripper = False
            self.closedGripper = False
            self.stateGripperInactive = True
            self.inactive_gripper_cb()
        else:
            pass

    def inactive_arm_cb(self):
        rospy.loginfo('%s: Inactive State. Press the left bottom to start.' % self._name)

    def inactive_gripper_cb(self):
        rospy.loginfo('%s: Press the right bottom to open the gripper.' % self._name)

    def teleop_cb(self):
        rospy.loginfo('%s: Tele-operation State. Press the left bottom to stop.' % self._name)
        # wait until actionlib server starts
        self.action_client_teleop.wait_for_server()
        # sends the goal to the action server
        self.action_client_teleop.send_goal(self.goal_teleop)

    def cmd_robotiq_cb(self, position):
        rospy.loginfo('%s: Commanding gripper position to %f.' % (self._name, position))
        # wait until actionlib server starts
        self.action_client_robotiq.wait_for_server()
        # update goal command
        self.goal_cmd_robotiq.position = position
        # sends the goal to the action server
        self.action_client_robotiq.send_goal(self.goal_cmd_robotiq)
        result = self.action_client_robotiq.wait_for_result()
        return result

    def cmd_arm_cb(self, pos_x_R, pos_y_R, pos_z_R, pos_x_L, pos_y_L, pos_z_L, string):
        rospy.loginfo('%s: Sending robot to %s.' % (self._name, string))
        # change possition
        self.goal_cmd_pose.poseR.position.x = pos_x_R
        self.goal_cmd_pose.poseR.position.y = pos_y_R
        self.goal_cmd_pose.poseR.position.z = pos_z_R
        self.goal_cmd_pose.poseL.position.x = pos_x_L
        self.goal_cmd_pose.poseL.position.y = pos_y_L
        self.goal_cmd_pose.poseL.position.z = pos_z_L
        # wait for server
        self.action_client_cmd_pose.wait_for_server()
        # send goal
        self.action_client_cmd_pose.send_goal_and_wait(self.goal_cmd_pose)
        # get result
        result = self.action_client_cmd_pose.get_result()
        return result.reached_goal

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('teleop_3d_client', anonymous=True)
    # Initialize node class
    cmd_config_client = TeleOp3DClient(rospy.get_name())
    # execute node
    rospy.spin()