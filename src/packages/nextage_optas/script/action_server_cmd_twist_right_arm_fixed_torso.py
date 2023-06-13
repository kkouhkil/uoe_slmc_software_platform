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
import sys
import numpy as np
np.set_printoptions(suppress=True)

import rospy
import actionlib
import optas

from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
# ROS messages types for the real robot
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
# ROS messages for command configuration action
from nextage_optas.msg import TriggerCmdAction, TriggerCmdFeedback, TriggerCmdResult

# For mux controller name
from std_msgs.msg import String
# service for selecting the controller
from topic_tools.srv import MuxSelect

from dynamic_reconfigure.server import Server
from nextage_optas.cfg import CmdTwistSingleArmConfig

class CmdTwistActionServer(object):
    """docstring for CmdTwistActionServer."""

    def __init__(self, name):
        # initialization message
        self.name = name
        rospy.loginfo("%s: Initializing class", self.name)
        #################################################################################
        ## get parameters:
        # qpoases regularisation eps
        self._eps_regularisation = rospy.get_param('~eps_regularisation', 0.01)
        # safety gains on joint position and velocity limits
        self._K_safety_lim_q = rospy.get_param('~K_safety_lim_q', 0.99)
        self._K_safety_lim_q_dot = rospy.get_param('~K_safety_lim_q', 0.6)
        # end-effector frame
        self._link_ee_right = rospy.get_param('~link_ee_right', 'link_ee_right')
        self._link_ee_left = rospy.get_param('~link_ee_left', 'link_ee_left')
        self._link_head = rospy.get_param('~link_head', 'link_head')
        self._link_ref = rospy.get_param('~link_ref', 'link_ref')
        self._link_gaze = rospy.get_param('~link_gaze', 'link_gaze')
        # control frequency
        self._freq = rospy.get_param('~freq', 100)
        self.dt = 1./self._freq
        # publishing command node name
        self._pub_cmd_topic_name = rospy.get_param('~cmd_topic_name', '/command')
        # teleop commands in 2d isometric
        self.joy_max = rospy.get_param('~joy_max', 0.68359375)
        # velocity scalling from the normalized value
        self.K_v_xy = rospy.get_param('~K_v_xy', 0.04)
        self.K_v_z = rospy.get_param('~K_v_z', 0.04)
        self.K_w_z = rospy.get_param('~K_w_z', 0.5)
        #################################################################################
        # initialize parameters
        self._pos_min = [0.0, 0.0, 0.0]
        self._pos_max = [0.0, 0.0, 0.0]
        self._ori_min = [0.0, 0.0, 0.0]
        self._ori_max = [0.0, 0.0, 0.0]
        self._W = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #################################################################################
        # initialize variables
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_z = 0.0
        self.w_z = 0.0
        #################################################################################
        # dynamic_reconfigure service
        Server(CmdTwistSingleArmConfig, self.dyn_param_cb)
        #################################################################################
        # load robot_description
        param_robot_description = rospy.get_param("~robot_description", "~/robot_description")
        if rospy.has_param(param_robot_description):
            self._robot_description = rospy.get_param(param_robot_description)
        else:
            rospy.logerr("%s: Param %s is unavailable!" % (self.name, param_robot_description))
            rospy.signal_shutdown('Incorrect parameter name.')
        #################################################################################
        ### optas
        # set up robot
        self.right_arm = optas.RobotModel(
            urdf_string=self._robot_description,
            time_derivs=[1],
            param_joints=['CHEST_JOINT0', 'HEAD_JOINT0', 'HEAD_JOINT1', 'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5'],
            name='nextage_right_arm'
        )
        self.right_arm_name = self.right_arm.get_name()
        self.ndof = self.right_arm.ndof
        # get right_arm limits
        q_min_right_arm = self._K_safety_lim_q * self.right_arm.dlim[0][0]
        self.q_min_right_arm = q_min_right_arm
        q_max_right_arm = self._K_safety_lim_q * self.right_arm.dlim[0][1]
        self.q_max_right_arm = q_max_right_arm
        dq_min_right_arm = self.dt * self._K_safety_lim_q_dot * self.right_arm.dlim[1][0]
        dq_max_right_arm = self.dt * self._K_safety_lim_q_dot * self.right_arm.dlim[1][1]
        # set up optimization builder
        builder_right_arm = optas.OptimizationBuilder(T=1, robots=[self.right_arm], derivs_align=True)
        # get right_arm joint variables
        dq_var = builder_right_arm.get_robot_states_and_parameters(self.right_arm_name, time_deriv=1)
        dq_opt = self.right_arm.extract_optimized_dimensions(dq_var)
        q_var = builder_right_arm.add_parameter('q', self.ndof)
        q_opt = self.right_arm.extract_optimized_dimensions(q_var)
        # set parameters
        dx = builder_right_arm.add_decision_variables('dx', 6)
        dx_target = builder_right_arm.add_parameter('dx_target', 6)
        W_x = builder_right_arm.add_parameter('W_x', 6)
        ori_min = builder_right_arm.add_parameter('ori_min', 3)
        ori_max = builder_right_arm.add_parameter('ori_max', 3)
        pos_min = builder_right_arm.add_parameter('pos_min', 3)
        pos_max = builder_right_arm.add_parameter('pos_max', 3)
        # kinematics
        fk = self.right_arm.get_global_link_position_function(link=self._link_ee_right)
        self.fk = fk
        rpy = self.right_arm.get_link_rpy_function(link=self._link_ee_right, base_link=self._link_ref)
        self.rpy = rpy
        J_pos = self.right_arm.get_global_link_linear_jacobian_function(link=self._link_ee_right)
        J_rpy = self.right_arm.get_link_angular_analytical_jacobian_function(link=self._link_ee_right, base_link=self._link_ref)
        # cost term
        builder_right_arm.add_cost_term('cost_q', optas.sumsqr(dq_opt))
        builder_right_arm.add_cost_term('cost_pos', optas.sumsqr(W_x * (dx_target-dx)))
        # forward differential kinematics
        builder_right_arm.add_equality_constraint('FDK_pos', (J_pos(q_var))@dq_var, dx[:3])
        builder_right_arm.add_equality_constraint('FDK_ori', (J_rpy(q_var))@dq_var, dx[3:])
        # add joint position limits
        builder_right_arm.add_bound_inequality_constraint('joint_pos_lim', q_min_right_arm, q_opt+dq_opt, q_max_right_arm)
        # add joint velocity limitis
        builder_right_arm.add_bound_inequality_constraint('joint_vel_lim', dq_min_right_arm, dq_opt, dq_max_right_arm)
        # add end-effector yaw-pitch-yaw limits
        builder_right_arm.add_bound_inequality_constraint('ori_lim', ori_min, rpy(q_var) + J_rpy(q_var)@dq_var, ori_max)
        # add workspace limits
        builder_right_arm.add_bound_inequality_constraint('pos_lim', pos_min, fk(q_var) + J_pos(q_var)@dq_var, pos_max)
        # setup solver
        self.solver_right_arm = optas.CasADiSolver(builder_right_arm.build()).setup(
            solver_name='qpoases',
            solver_options={'error_on_fail': False, 'enableRegularisation': True, 'epsRegularisation': self._eps_regularisation}
        )
        #################################################################################
        # initialize variables
        self.q_read = np.zeros(self.ndof)
        self.q_cmd = np.zeros(self.ndof)
        #################################################################################
        # declare joint subscriber
        self._joint_sub = rospy.Subscriber(
            "/joint_states",
            JointState,
            self.read_joint_states_cb
        )
        # declare joystick subscriver
        self._joy_sub = rospy.Subscriber(
            "/twist_command",
            Twist,
            self.read_twist_cb
        )
        # declare joint publisher
        self._joint_pub = rospy.Publisher(
            self._pub_cmd_topic_name,
            Float64MultiArray,
            queue_size=10
        )
        #################################################################################
        # set mux controller selection as wrong by default
        self._correct_mux_selection = False
        # declare mux service
        self._srv_mux_sel = rospy.ServiceProxy(rospy.get_namespace() + '/mux_joint_position/select', MuxSelect)
        # declare subscriber for selected controller
        self._sub_selected_controller = rospy.Subscriber(
            "/mux_selected",
            String,
            self.read_mux_selection
        )
        #################################################################################
        # initialize action messages
        self._feedback = TriggerCmdFeedback()
        self._result = TriggerCmdResult()
        # declare action server
        self._action_server = actionlib.SimpleActionServer(
            "cmd_twist_fixed_torso", 
            TriggerCmdAction, 
            execute_cb=None,
            auto_start=False
        )
        # register the preempt callback
        self._action_server.register_goal_callback(self.goal_cb)
        self._action_server.register_preempt_callback(self.preempt_cb)
        # start action server
        self._action_server.start()
        #################################################################################

    def goal_cb(self):
        # activate publishing command
        self._srv_mux_sel(self._pub_cmd_topic_name)
        # accept the new goal request
        acceped_goal = self._action_server.accept_new_goal()
        self.target_pos_z = acceped_goal.pos_z
        ### ---------------------------------------------------------
        # initialize the message
        self._msg = Float64MultiArray()
        self._msg.layout = MultiArrayLayout()
        self._msg.layout.data_offset = 0
        self._msg.layout.dim.append(MultiArrayDimension())
        self._msg.layout.dim[0].label = "columns"
        self._msg.layout.dim[0].size = self.ndof
        # read current robot joint positions for memory
        self.q_cmd = self.q_read
        # create timer
        dur = rospy.Duration(1.0/self._freq)
        self._timer = rospy.Timer(dur, self.timer_cb)

    def timer_cb(self, event):
        """ Publish the robot configuration """

        # make sure that the action is active
        if(not self._action_server.is_active()):
            self._timer.shutdown()
            rospy.logwarn("%s: The action server is NOT active!")
            self._result.trigger_off = False
            self._action_server.set_aborted(self._result)
            return

        # main execution
        if(self._correct_mux_selection):
            # current config
            q_curr = self.q_cmd
            # target displacement
            dx_target = [
                self.dt * self.v_x,
                self.dt * self.v_y,
                self.dt * self.v_z,
                0.0,
                0.0,
                self.dt * self.w_z,
            ]
            dq = optas.DM.zeros(self.ndof)
            ### ---------------------------------------------------------
            ### right hand problem
            self.solver_right_arm.reset_initial_seed({f'{self.right_arm_name}/dq/x': self.right_arm.extract_optimized_dimensions(dq)})
            self.solver_right_arm.reset_parameters({
                f'{self.right_arm_name}/dq/p': self.right_arm.extract_parameter_dimensions(dq),
                'q': q_curr,
                'dx_target': dx_target,
                'pos_min': self._pos_min,
                'pos_max': self._pos_max,
                'ori_min': self._ori_min,
                'ori_max': self._ori_max,
                'W_x': self._W,
            })
            # solve problem
            solution = self.solver_right_arm.solve()
            if self.solver_right_arm.did_solve():
                dq = solution[f'{self.right_arm_name}/dq']
            else:
                rospy.logwarn("%s: Right arm QP fail to find a solution!" % self.name)
            ### ---------------------------------------------------------
            # numpy dq
            dq = np.asarray(dq).T[0]
            # integrate solution
            self.q_cmd = q_curr + dq
            # update message
            self._msg.data = self.q_cmd
            # publish message
            self._joint_pub.publish(self._msg)
            # compute progress
            self._feedback.is_active = True
            # publish feedback
            self._action_server.publish_feedback(self._feedback)
        else:
            # shutdown this timer
            self._timer.shutdown()
            rospy.logwarn("%s: Request aborted. The controller selection changed!" % (self.name))
            self._result.trigger_off = False
            self._action_server.set_aborted(self._result)
            return

    def read_twist_cb(self, msg):
        # read planar part of the twist
        v_x = msg.linear.x
        v_y = msg.linear.y
        v_z = msg.linear.z
        w_z = msg.angular.z
        # make v_x and v_y isometric
        v_ang = np.arctan2(v_y, v_x)
        c_ang = np.cos(v_ang)
        s_ang = np.sin(v_ang)
        if v_ang > np.deg2rad(-45.) and v_ang < np.deg2rad(45.):
            norm_max = self.joy_max/c_ang
        elif v_ang > np.deg2rad(45.) and v_ang < np.deg2rad(135.):
            norm_max = self.joy_max/s_ang
        elif v_ang < np.deg2rad(-45.) and v_ang > np.deg2rad(-135.):
            norm_max = -self.joy_max/s_ang
        else:
            norm_max = -self.joy_max/c_ang
        # save values
        self.v_x =   self.K_v_xy * (v_x / norm_max)
        self.v_y =   self.K_v_xy * (v_y / norm_max)
        self.v_z =   self.K_v_z * (v_z / self.joy_max)
        self.w_z = - self.K_w_z * (w_z / self.joy_max)

    def read_joint_states_cb(self, msg):
        self.q_read = np.asarray(list(msg.position))

    def read_mux_selection(self, msg):
        self._correct_mux_selection = (msg.data == self._pub_cmd_topic_name)

    def preempt_cb(self):
        self._timer.shutdown()
        rospy.loginfo("%s: Client preempted this action.", self.name)
        self._result.trigger_off = True
        # set the action state to preempted
        self._action_server.set_preempted(self._result)

    def dyn_param_cb(self, config, level):
        rospy.loginfo("%s: Reconfigure request for dynamic parameters." % (self.name))
        # get new parameters
        self._pos_min = [config.x_min, config.y_min, config.z_min]
        self._pos_max = [config.x_max, config.y_max, config.z_max]
        self._ori_min = np.deg2rad([config.roll_min, config.pitch_min, config.yaw_min])
        self._ori_max = np.deg2rad([config.roll_max, config.pitch_max, config.yaw_max])
        self._W = [config.W_x, config.W_y, config.W_z, config.W_roll, config.W_pitch, config.W_yaw]
        return config

if __name__=="__main__":
    # Initialize node
    rospy.init_node("teleop_3d_server", anonymous=True)
    # Initialize node class
    cmd_twist_server = CmdTwistActionServer(rospy.get_name())
    # executing node
    rospy.spin()
