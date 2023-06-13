#! /usr/bin/env python3

# Copyright (C) 2022 Statistical Machine Learning and Motor Control Group (SLMC)
# Authors: Joao Moura (maintainer)
# email: joao.moura@ed.ac.uk

# This file is part of iiwa_wiping package.

# iiwa_wiping is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# iiwa_wiping is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
import numpy as np
np.set_printoptions(suppress=True)

import rospy
import actionlib
import optas

from sensor_msgs.msg import JointState
# ROS messages types for the real robot
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
# ROS messages for command configuration action
from nextage_optas.msg import CmdDualPoseAction, CmdDualPoseFeedback, CmdDualPoseResult

# For mux controller name
from std_msgs.msg import String
# service for selecting the controller
from topic_tools.srv import MuxSelect

class CmdPoseActionServer(object):
    """docstring for CmdPoseActionServer."""

    def __init__(self, name):
        # initialization message
        self._name = name
        rospy.loginfo("%s: Initializing class", self._name)
        ## get parameters:
        ## --------------------------------------
        # workspace limit boundaries
        self._x_min = rospy.get_param('~x_min', 0.2)
        self._x_max = rospy.get_param('~x_max', 0.8)
        self._y_min = rospy.get_param('~y_min', -0.5)
        self._y_max = rospy.get_param('~y_max', 0.5)
        self._z_min = rospy.get_param('~z_min', 0.75)
        self._z_max = rospy.get_param('~z_max', 1.5)
        self._pos_min = np.asarray([self._x_min, self._y_min, self._z_min])
        self._pos_max = np.asarray([self._x_max, self._y_max, self._z_max])
        # robot name
        # end-effector frame
        self._link_ee_right = rospy.get_param('~link_ee_right', 'link_ee_right')
        self._link_ee_left = rospy.get_param('~link_ee_left', 'link_ee_left')
        self._link_head = rospy.get_param('~link_head', 'link_head')
        self._link_gaze = rospy.get_param('~link_gaze', 'link_gaze')
        # control frequency
        self._freq = rospy.get_param('~freq', 100)
        # publishing command node name
        self._pub_cmd_topic_name = rospy.get_param('~cmd_topic_name', '/command')
        # load robot_description
        param_robot_description = rospy.get_param("~robot_description", "~/robot_description")
        if rospy.has_param(param_robot_description):
            self._robot_description = rospy.get_param(param_robot_description)
        else:
            rospy.logerr("%s: Param %s is unavailable!" % (self._name, param_robot_description))
            rospy.signal_shutdown('Incorrect parameter name.')
        ### optas
        ### ---------------------------------------------------------
        # set up right arm optimizataion
        self.right_arm = optas.RobotModel(
            urdf_string=self._robot_description,
            time_derivs=[0],
            param_joints=['HEAD_JOINT0', 'HEAD_JOINT1', 'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5'],
            name='nextage_right_arm'
        )
        self.right_arm_name = self.right_arm.get_name()
        self.ndof = self.right_arm.ndof
        # nominal robot configuration
        q_nom = optas.DM.zeros(self.ndof)
        # set up optimization builder
        builder_right_arm = optas.OptimizationBuilder(T=1, robots=[self.right_arm])
        # get robot state and parameters variables
        q_var = builder_right_arm.get_robot_states_and_parameters(self.right_arm_name)
        # get end-effector pose as parameters
        pos = builder_right_arm.add_parameter('pos', 3)
        ori = builder_right_arm.add_parameter('ori', 4)
        # set variable boudaries
        builder_right_arm.enforce_model_limits(self.right_arm_name)
        # equality constraint on right arm position
        pos_fnc = self.right_arm.get_global_link_position_function(link=self._link_ee_right)
        builder_right_arm.add_equality_constraint('final_pos', pos_fnc(q_var), rhs=pos)
        # rotation of the right arm position
        self.R_fnc = self.right_arm.get_global_link_rotation_function(link=self._link_ee_right)
        # equality constraint on orientation
        ori_fnc = self.right_arm.get_global_link_quaternion_function(link=self._link_ee_right)
        builder_right_arm.add_equality_constraint('final_ori', ori_fnc(q_var), rhs=ori)
        # optimization cost: close to nominal config
        builder_right_arm.add_cost_term('nom_config', optas.sumsqr(q_var-q_nom))
        # setup solver
        self.solver_right_arm = optas.CasADiSolver(optimization=builder_right_arm.build()).setup('ipopt')
        ### ---------------------------------------------------------
        # set up head optimization
        self.head = optas.RobotModel(
            urdf_string=self._robot_description,
            time_derivs=[0],
            param_joints=['CHEST_JOINT0', 'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5'],
            name='nextage_head'
        )
        self.head_name = self.head.get_name()
        # set up optimization builder
        builder_head = optas.OptimizationBuilder(T=1, robots=[self.head])
        # get robot state and parameters variables
        q_var = builder_head.get_robot_states_and_parameters(self.head_name)
        # get end-effector pose as parameters
        pos = builder_head.add_parameter('pos', 3)
        # get head heading
        pos_head = self.head.get_global_link_position_function(link=self._link_head)
        # get gaze position
        self.pos_gaze_fnc = self.head.get_global_link_position_function(link=self._link_gaze)
        # set variable boudaries
        builder_head.enforce_model_limits(self.head_name)
        # optimization cost: close to nominal config
        builder_head.add_cost_term('heading', optas.norm_2(pos_head(q_var)-pos))
        # setup solver
        self.solver_head = optas.CasADiSolver(optimization=builder_head.build()).setup('ipopt')
        ### ---------------------------------------------------------
        # set up left arm optimizataion
        self.left_arm = optas.RobotModel(
            urdf_string=self._robot_description,
            time_derivs=[0],
            param_joints=['CHEST_JOINT0', 'HEAD_JOINT0', 'HEAD_JOINT1', 'RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5'],
            name='nextage_left_arm'
        )
        self.left_arm_name = self.left_arm.get_name()
        # set up optimization builder
        builder_left_arm = optas.OptimizationBuilder(T=1, robots=[self.left_arm])
        # get robot state and parameters variables
        q_var = builder_left_arm.get_robot_states_and_parameters(self.left_arm_name)
        q_opt = builder_left_arm.get_model_states(self.left_arm_name)
        # get end-effector pose as parameters
        pos = builder_left_arm.add_parameter('pos', 3)
        # set variable boudaries
        builder_left_arm.enforce_model_limits(self.left_arm_name)
        # equality constraint on position
        pos_fnc = self.left_arm.get_global_link_position_function(link=self._link_ee_left)
        builder_left_arm.add_equality_constraint('final_pos', pos_fnc(q_var), rhs=pos)
        # optimization cost: close to nominal config
        builder_left_arm.add_cost_term('nom_config', optas.sumsqr(q_opt-self.left_arm.extract_optimized_dimensions(q_nom)))
        # setup solver
        self.solver_left_arm = optas.CasADiSolver(optimization=builder_left_arm.build()).setup('ipopt')
        ### ---------------------------------------------------------
        # initialize variables
        self.q_curr = np.zeros(self.ndof)
        self.joint_names = []
        # declare joint subscriber
        self._joint_sub = rospy.Subscriber(
            "/joint_states",
            JointState,
            self.read_joint_states_cb
        )
        # declare joint publisher
        self._joint_pub = rospy.Publisher(
            self._pub_cmd_topic_name,
            Float64MultiArray,
            queue_size=10
        )
        # set mux controller selection as wrong by default
        self._correct_mux_selection = False
        # declare mux service
        self._srv_mux_sel = rospy.ServiceProxy(rospy.get_namespace() + "/mux_joint_position/select", MuxSelect)
        # declare subscriber for selected controller
        self._sub_selected_controller = rospy.Subscriber(
            "/mux_selected",
            String,
            self.read_mux_selection
        )
        # initialize action messages
        self._feedback = CmdDualPoseFeedback()
        self._result = CmdDualPoseResult()
        # declare action server
        self._action_server = actionlib.SimpleActionServer(
            'cmd_pose', 
            CmdDualPoseAction, 
            execute_cb=None,
            auto_start=False
        )
        # register the preempt callback
        self._action_server.register_goal_callback(self.goal_cb)
        self._action_server.register_preempt_callback(self.preempt_cb)
        # start action server
        self._action_server.start()

    def goal_cb(self):
        # activate publishing command
        self._srv_mux_sel(self._pub_cmd_topic_name)
        # accept the new goal request
        acceped_goal = self._action_server.accept_new_goal()
        # desired end-effector position
        pos_R = np.asarray([
                acceped_goal.poseR.position.x,
                acceped_goal.poseR.position.y,
                acceped_goal.poseR.position.z
        ])
        pos_L = np.asarray([
                acceped_goal.poseL.position.x,
                acceped_goal.poseL.position.y,
                acceped_goal.poseL.position.z
        ])
        ori_T = np.asarray([
                acceped_goal.poseR.orientation.x,
                acceped_goal.poseR.orientation.y,
                acceped_goal.poseR.orientation.z,
                acceped_goal.poseR.orientation.w
        ])
        # check boundaries of the position
        if (pos_R > self._pos_max).any() or (pos_R < self._pos_min).any():
            rospy.logwarn("%s: Request aborted. Goal position (%.2f, %.2f, %.2f) is outside of the workspace boundaries. Check parameters for this node." % (self._name, pos_R[0], pos_R[1], pos_R[2]))
            self._result.reached_goal = False
            self._action_server.set_aborted(self._result)
            return
        if (pos_L > self._pos_max).any() or (pos_L < self._pos_min).any():
            rospy.logwarn("%s: Request aborted. Goal position (%.2f, %.2f, %.2f) is outside of the workspace boundaries. Check parameters for this node." % (self._name, pos_L[0], pos_L[1], pos_L[2]))
            self._result.reached_goal = False
            self._action_server.set_aborted(self._result)
            return
        # print goal request
        rospy.loginfo("%s: Request to send right arm to position (%.2f, %.2f, %.2f) with orientation (%.2f, %.2f, %.2f, %.2f), and left arm to (%.2f, %.2f, %.2f) in %.1f seconds." % (
                self._name, 
                pos_R[0], pos_R[1], pos_R[2],
                ori_T[0], ori_T[1], ori_T[2], ori_T[3],
                pos_L[0], pos_L[1], pos_L[2],
                acceped_goal.duration
            )
        )
        # read current robot joint positions
        q0 = self.q_curr
        qT = optas.DM.zeros(self.ndof)
        ### optas
        ### ---------------------------------------------------------
        ### right hand problem
        # set initial seed
        self.solver_right_arm.reset_initial_seed({
            f'{self.right_arm_name}/q/x': self.right_arm.extract_optimized_dimensions(qT),
        })
        self.solver_right_arm.reset_parameters({
            'pos': pos_R,
            'ori': ori_T,
            f'{self.right_arm_name}/q/p': self.right_arm.extract_parameter_dimensions(qT),
        })
        # solve problem
        solution = self.solver_right_arm.solve()
        qT = solution[f'{self.right_arm_name}/q']
        ### ---------------------------------------------------------
        ### head problem
        # set initial seed
        self.solver_head.reset_initial_seed({
            f'{self.head_name}/q/x': self.head.extract_optimized_dimensions(qT),
        })
        self.solver_head.reset_parameters({
            'pos': self.pos_gaze_fnc(qT),
            f'{self.head_name}/q/p': self.head.extract_parameter_dimensions(qT),
        })
        # solve problem
        solution = self.solver_head.solve()
        qT = solution[f'{self.head_name}/q']
        ### ---------------------------------------------------------
        ### left arm problem
        # set initial seed
        self.solver_left_arm.reset_initial_seed({
            f'{self.left_arm_name}/q/x': self.left_arm.extract_optimized_dimensions(qT),
        })
        self.solver_left_arm.reset_parameters({
            'pos': pos_L,
            f'{self.left_arm_name}/q/p': self.left_arm.extract_parameter_dimensions(qT),
        })
        # solve problem
        solution = self.solver_left_arm.solve()
        qT = solution[f'{self.left_arm_name}/q']
        ### ---------------------------------------------------------
        # helper variables
        T = acceped_goal.duration
        # print goal request
        qT = np.asarray(qT).T[0]
        rospy.loginfo("%s: Request to send robot joints to %s in %.1f seconds." % (self._name, qT, T))
        self._steps = int(T * self._freq)
        self._idx = 0
        Dq = qT - q0
        # interpolate between current and target configuration 
        # polynomial obtained for zero speed (3rd order) and acceleratin (5th order)
        # at the initial and final time
        self._q = lambda t: q0 + (3.*((t/T)**2) - 2.*((t/T)**3))*Dq # 3rd order
        # self._q = lambda t: q0 + (10.*((t/T)**3) - 15.*((t/T)**4) + 6.*((t/T)**5))*Dq # 5th order
        self._t = np.linspace(0., T, self._steps + 1)
        # initialize the message
        self._msg = Float64MultiArray()
        self._msg.layout = MultiArrayLayout()
        self._msg.layout.data_offset = 0
        self._msg.layout.dim.append(MultiArrayDimension())
        self._msg.layout.dim[0].label = "columns"
        self._msg.layout.dim[0].size = self.ndof
        # create timer
        dur = rospy.Duration(1.0/self._freq)
        self._timer = rospy.Timer(dur, self.timer_cb)

    def timer_cb(self, event):
        """ Publish the robot configuration """

        # make sure that the action is active
        if(not self._action_server.is_active()):
            self._timer.shutdown()
            rospy.logwarn("%s: The action server is NOT active!")
            self._result.reached_goal = False
            self._action_server.set_aborted(self._result)
            return

        # main execution
        if(self._idx < self._steps):
            if(self._correct_mux_selection):
                # increment idx (in here counts starts with 1)
                self._idx += 1
                # compute next configuration with lambda function
                q_next = self._q(self._t[self._idx])
                # update message
                self._msg.data = q_next
                # publish message
                self._joint_pub.publish(self._msg)
                # compute progress
                self._feedback.progress = (self._idx*100)/self._steps
                # publish feedback
                self._action_server.publish_feedback(self._feedback)
            else:
                # shutdown this timer
                self._timer.shutdown()
                rospy.logwarn("%s: Request aborted. The controller selection changed!" % (self._name))
                self._result.reached_goal = False
                self._action_server.set_aborted(self._result)
                return
        else:
            # shutdown this timer
            self._timer.shutdown()
            # set the action state to succeeded
            rospy.loginfo("%s: Succeeded" % self._name)
            self._result.reached_goal = True
            self._action_server.set_succeeded(self._result)
            return

    def read_joint_states_cb(self, msg):
        self.q_curr = np.asarray(list(msg.position))
        self.joint_names = msg.name

    def read_mux_selection(self, msg):
        self._correct_mux_selection = (msg.data == self._pub_cmd_topic_name)

    def preempt_cb(self):
        rospy.loginfo("%s: Preempted.", self._name)
        # set the action state to preempted
        self._action_server.set_preempted()

if __name__=="__main__":
    # Initialize node
    rospy.init_node("cmd_pose_server", anonymous=True)
    # Initialize node class
    cmd_pose_server = CmdPoseActionServer(rospy.get_name())
    # executing node
    rospy.spin()