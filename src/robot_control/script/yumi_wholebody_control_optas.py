#! /usr/bin/env python3

import sys
import os
import numpy as np
import argparse
import time
import pathlib
import actionlib
import rospy
import optas
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import tf

from collections import namedtuple
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from pushing_msgs.msg import CmdYumiPoseGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from urdf_parser_py.urdf import URDF

class CmdYumiPoseClient(object):

    def __init__(self, name, target_pos_YumiBase, target_quat_YumiBase, target_pos_YumiR, target_quat_YumiR, target_pos_YumiL, target_quat_YumiL, duration) -> None:

        # initialization message
        self.name = name
        self.target_pos_YumiBase = target_pos_YumiBase
        self.target_quat_YumiBase = target_quat_YumiBase
        self.target_pos_YumiR = target_pos_YumiR
        self.target_quat_YumiR = target_quat_YumiR
        self.target_pos_YumiL = target_pos_YumiL
        self.target_quat_YumiL = target_quat_YumiL
        self.duration = duration

        # creates goal and sends to the action server
        goal = CmdYumiPoseGoal()
        goal.poseYumiBase.position.x = self.target_pos_YumiBase[0]
        goal.poseYumiBase.position.y = self.target_pos_YumiBase[1]
        goal.poseYumiBase.position.z = self.target_pos_YumiBase[2]
        goal.poseYumiBase.orientation.x = self.target_quat_YumiBase[0]
        goal.poseYumiBase.orientation.y = self.target_quat_YumiBase[1]
        goal.poseYumiBase.orientation.z = self.target_quat_YumiBase[2]
        goal.poseYumiBase.orientation.w = self.target_quat_YumiBase[3]
        goal.poseYumiR.position.x = self.target_pos_YumiR[0]
        goal.poseYumiR.position.y = self.target_pos_YumiR[1]
        goal.poseYumiR.position.z = self.target_pos_YumiR[2]
        goal.poseYumiR.orientation.x = self.target_quat_YumiR[0]
        goal.poseYumiR.orientation.y = self.target_quat_YumiR[1]
        goal.poseYumiR.orientation.z = self.target_quat_YumiR[2]
        goal.poseYumiR.orientation.w = self.target_quat_YumiR[3]
        goal.poseYumiL.position.x = self.target_pos_YumiL[0]
        goal.poseYumiL.position.y = self.target_pos_YumiL[1]
        goal.poseYumiL.position.z = self.target_pos_YumiL[2]
        goal.poseYumiL.orientation.x = self.target_quat_YumiL[0]
        goal.poseYumiL.orientation.y = self.target_quat_YumiL[1]
        goal.poseYumiL.orientation.z = self.target_quat_YumiL[2]
        goal.poseYumiL.orientation.w = self.target_quat_YumiL[3]
        goal.duration = self.duration

class CmdYumiRobotControl(object):

    def __init__(self, name):
        
        # Initialization of reqruied variables
        self.name = name  
        self.duration_coef = 1
        self.vec_count = 0

        self.yumi_left_arm_pos = [0] * 7
        self.yumi_right_arm_pos = [0] * 7

        self.YUMI_LEFT_ARM_HOME_POSITION = [-0.523, -1.309, 1.0472, 0.0174, 0.523, 0.785, -1.570]
        self.YUMI_RIGHT_ARM_HOME_POSITION = [0.523, -1.309, -1.0472, -0.0174, -0.523, 0.785, -1.570]

        self.yumi_left_arm_pos = self.YUMI_LEFT_ARM_HOME_POSITION 
        self.yumi_right_arm_pos = self.YUMI_RIGHT_ARM_HOME_POSITION
        
        self.q_curr_joint_global = [0] * 14
        self.q_dot_curr_joint_global = [0] * 14

        self.q_curr_base_global = [0] * 3
        self.q_dot_curr_base_global = [0] * 3

        self.q_des_base_global = [0] * 3
        self.q_tilde_base_global = [0] * 3

        self.yumi_base_msg = Twist()

        # Stiffness profile for base motion
        self.K_Px_const = 0.75
        self.K_Py_const = 0.75
        self.K_Rz_const = 0.75

        self.K_Px_total = 0
        self.K_Py_total = 0
        self.K_Rz_total = 0

        # Constant Damping for base motion
        self.D_Px_const = 0.1
        self.D_Py_const = 0.1
        self.D_Rz_const = 0.1

        self.D_Px_total = 0
        self.D_Py_total = 0
        self.D_Rz_total = 0

        self.pol_traj_global_duration_time = 0
        self.pol_traj_global_num_steps = 0

        self.qT_array_pol_traj_list = []

        # rospy
        rospy.loginfo("%s: Initializing Yumi Robot Control class", self.name)

        # Control frequency
        self.freq = rospy.get_param('~freq', 100)

        # Specify URDF filename
        cwd = pathlib.Path(__file__).parent.resolve()  # path to current working directory
        urdf_filename = os.path.join(
            cwd, "robots", "yumi_mobile_base_urdf", "yumi_mobile_base_urdf.urdf"
        )  # Yumi ABB Robot - Dual-arm mobile based

        # Setup robot model
        self.yumi_robot = optas.RobotModel(urdf_filename=urdf_filename,
                                 time_derivs = [0],
                                 param_joints = [],
                                 name = 'yumi_abb_robot')
        
        # Getting the number of actuated joints for Yumi base and 2 7-DoF arms
        self.yumi_robot_actuated_joint_names = self.yumi_robot.actuated_joint_names
        self.yumi_robot_actuated_joint_ndof = len(self.yumi_robot_actuated_joint_names)
        self.qT_array = [0] * self.yumi_robot_actuated_joint_ndof
        print("\nYumi Robot - Actuated Joint Names: \n", self.yumi_robot_actuated_joint_names)
        print("\nYumi Robot - nDoF = ", len(self.yumi_robot_actuated_joint_names))

        # Getting robot name and number of degress of freedom
        self.yumi_robot_name = self.yumi_robot.get_name()
        self.yumi_robot_dim = self.yumi_robot.get_dim()
        self.yumi_robot_ndof = self.yumi_robot_actuated_joint_ndof
        self.yumi_ndof_base = 3

        # Nominal robot configuration
        self.q_nom = optas.DM.zeros(self.yumi_robot_ndof)
        self.yumi_robot_opt_idx = self.yumi_robot.optimized_joint_indexes[:]
        self.yumi_robot_param_idx = self.yumi_robot.parameter_joint_indexes

        # Set up optimization builder
        T = 1
        self.builder_yumi_robot = optas.OptimizationBuilder(T = T, robots = [self.yumi_robot])

        # Get robot state and parameters variables
        q_var = self.builder_yumi_robot.get_robot_states_and_parameters(self.yumi_robot_name)

        # Get end-effector pose as parameters
        pos_R = self.builder_yumi_robot.add_parameter('pos_R', 3)
        ori_R = self.builder_yumi_robot.add_parameter('ori_R', 4)
        pos_L = self.builder_yumi_robot.add_parameter('pos_L', 3)
        ori_L = self.builder_yumi_robot.add_parameter('ori_L', 4)

        # Set variable boudaries
        self.builder_yumi_robot.enforce_model_limits(self.yumi_robot_name)

        # Equality constraint on right and left arm positions
        self.end_effector_name_left = "yumi_robl_link_7"        
        self.end_effector_name_right = "yumi_robr_link_7"
        pos_fnc_Left = self.yumi_robot.get_global_link_position_function(link = self.end_effector_name_left)
        pos_fnc_Right = self.yumi_robot.get_global_link_position_function(link = self.end_effector_name_right)
        self.builder_yumi_robot.add_equality_constraint('final_pos_Left', pos_fnc_Left(q_var), rhs = pos_L)
        self.builder_yumi_robot.add_equality_constraint('final_pos_Right', pos_fnc_Right(q_var), rhs = pos_R)

        # Rotation of the right and left arms
        self.Rotation_fnc_Left = self.yumi_robot.get_global_link_rotation_function(link = self.end_effector_name_left)        
        self.Rotation_fnc_Right = self.yumi_robot.get_global_link_rotation_function(link = self.end_effector_name_right)

        # equality constraint on orientations
        ori_fnc_Left = self.yumi_robot.get_global_link_quaternion_function(link = self.end_effector_name_left)        
        ori_fnc_Right = self.yumi_robot.get_global_link_quaternion_function(link = self.end_effector_name_right)
        self.builder_yumi_robot.add_equality_constraint('final_ori_Left', ori_fnc_Left(q_var), rhs = ori_L)
        self.builder_yumi_robot.add_equality_constraint('final_ori_Right', ori_fnc_Right(q_var), rhs = ori_R)

        # optimization cost: close to nominal config
        self.builder_yumi_robot.add_cost_term('nom_config', 0 * optas.sumsqr(q_var - self.q_nom))

        # Setup solver
        self.solver_yumi_robot = optas.CasADiSolver(optimization=self.builder_yumi_robot.build()).setup('ipopt')

        # Declaration of yumi-base subscriber
        self.joint_sub_yumi_base = rospy.Subscriber(
            "/odom",
            Odometry,
            self.read_base_states_cb
        )

        # Declaration of yumi-base publisher
        self.joint_pub_yumi_base_velocity = rospy.Publisher(
            "/four_wheel_controller/cmd_vel",
            Twist,
            queue_size=10
        )

        # Declaration of yumi-joint subscriber
        self.joint_sub = rospy.Subscriber(
            "/joint_states",
            JointState,
            self.read_joint_states_cb
        )

        # Declaration of yumi-joint publisher
        self.yumi_robl_arm_joint_pub1 = rospy.Publisher('/robl_joint_1_position_controller/command', Float64, queue_size=10)
        self.yumi_robl_arm_joint_pub2 = rospy.Publisher('/robl_joint_2_position_controller/command', Float64, queue_size=10)
        self.yumi_robl_arm_joint_pub3 = rospy.Publisher('/robl_joint_3_position_controller/command', Float64, queue_size=10)
        self.yumi_robl_arm_joint_pub4 = rospy.Publisher('/robl_joint_4_position_controller/command', Float64, queue_size=10)
        self.yumi_robl_arm_joint_pub5 = rospy.Publisher('/robl_joint_5_position_controller/command', Float64, queue_size=10)
        self.yumi_robl_arm_joint_pub6 = rospy.Publisher('/robl_joint_6_position_controller/command', Float64, queue_size=10)
        self.yumi_robl_arm_joint_pub7 = rospy.Publisher('/robl_joint_7_position_controller/command', Float64, queue_size=10)

        self.yumi_robr_arm_joint_pub1 = rospy.Publisher('/robr_joint_1_position_controller/command', Float64, queue_size=10)
        self.yumi_robr_arm_joint_pub2 = rospy.Publisher('/robr_joint_2_position_controller/command', Float64, queue_size=10)
        self.yumi_robr_arm_joint_pub3 = rospy.Publisher('/robr_joint_3_position_controller/command', Float64, queue_size=10)
        self.yumi_robr_arm_joint_pub4 = rospy.Publisher('/robr_joint_4_position_controller/command', Float64, queue_size=10)
        self.yumi_robr_arm_joint_pub5 = rospy.Publisher('/robr_joint_5_position_controller/command', Float64, queue_size=10)
        self.yumi_robr_arm_joint_pub6 = rospy.Publisher('/robr_joint_6_position_controller/command', Float64, queue_size=10)
        self.yumi_robr_arm_joint_pub7 = rospy.Publisher('/robr_joint_7_position_controller/command', Float64, queue_size=10)

        # Declaration of end_effector(s) subscriber
        self.yumi_robl_arm_end_eff = tf.TransformListener()
        self.yumi_robr_arm_end_eff = tf.TransformListener()

    def pybullet_interfacce():
    #     # Yumi dual arm optimization
    #     self.simulationStepTime = 0.005
    #     self.vis = False

    #     p.connect(p.GUI if self.vis else p.DIRECT)
    #     p.setAdditionalSearchPath(pybullet_data.getDataPath())
    #     p.setGravity(0, 0, -9.81)
    #     p.setTimeStep(self.simulationStepTime)
    #     self.plane_id = p.loadURDF('plane.urdf')
    #     self.load_robot(urdf = '/home/keyhan/yumi_abb/src/ABB-YuMi_Complete_Model/urdfs/yumi_grippers.urdf',print_joint_info=True)

    # def load_robot (self,urdf, print_joint_info = False):        
    #     self.robot_id = p.loadURDF(urdf,[0, 0, -0.11], [0, 0, 0, 1])
    #     numJoints = p.getNumJoints(self.robot_id)
    #     jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
    #     controlJoints = ["yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
    #                      "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r","gripper_r_joint","gripper_r_joint_m",
    #                      "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
    #                      "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l","gripper_l_joint","gripper_l_joint_m"]
    #     self._left_ee_frame_name  = 'yumi_joint_6_l'
    #     self._right_ee_frame_name = 'yumi_joint_6_r'
    #     self._LEFT_HOME_POSITION = self.YUMI_LEFT_ARM_HOME_POSITION
    #     self._RIGHT_HOME_POSITION = self.YUMI_RIGHT_ARM_HOME_POSITION
    #     self._RIGHT_HAND_JOINT_IDS = [1,2,3,4,5,6,7]
    #     self._RIGHT_GRIP_JOINT_IDS = [9,10]
    #     self._LEFT_HAND_JOINT_IDS = [11,12,13,14,15,16,17]
    #     self._LEFT_GRIP_JOINT_IDS = [19,20]
    #     self._max_torques = [42, 90, 39, 42, 3, 12, 1]
    #     self._jointInfo = namedtuple("jointInfo",
    #                        ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity",
    #                         "controllable", "jointAxis", "parentFramePos", "parentFrameOrn"])
    #     self._joint_Damping =  [0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 
    #                             0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005]
    #     for i in range(numJoints):
    #         info = p.getJointInfo(self.robot_id, i)
    #         jointID = info[0]
    #         jointName = info[1].decode("utf-8")
    #         jointType = jointTypeList[info[2]]
    #         jointLowerLimit = info[8]
    #         jointUpperLimit = info[9]
    #         jointMaxForce = info[10]
    #         jointMaxVelocity = info[11]
    #         jointAxis = info[13]
    #         parentFramePos = info[14]
    #         parentFrameOrn = info[15]
    #         controllable = True if jointName in controlJoints else False
    #         info = self._jointInfo(jointID, jointName, jointType, jointLowerLimit,
    #                         jointUpperLimit, jointMaxForce, jointMaxVelocity, controllable,
    #                         jointAxis, parentFramePos, parentFrameOrn)
    #         if info.type == "REVOLUTE" or info.type == "PRISMATIC":  # set revolute joint to static
    #             p.setJointMotorControl2(self.robot_id, info.id, p.POSITION_CONTROL, targetPosition=0, force=0)
    #             # if print_joint_info:
    #             #     print (info)
    #             #     print (jointType)                            
    #             #     print ('-'*40)

    # @staticmethod
    # def ang_in_mpi_ppi(angle):
    #     """
    #     Convert the angle to the range [-pi, pi).

    #     Args:
    #         angle (float): angle in radians.

    #     Returns:
    #         float: equivalent angle in [-pi, pi).
    #     """

    #     angle = (angle + np.pi) % (2 * np.pi) - np.pi
    #     return angle            

    # def get_left_ee_state(self):
    #     return p.getLinkState(self.robot_id,self._left_ee_frame_name)
    
    # def get_right_ee_state(self):
    #     return p.getLinkState(self.robot_id,self._right_ee_frame_name)   

    # def move_left_arm(self,pose):                
    #     joint_poses = p.calculateInverseKinematics(self.robot_id, self._LEFT_HAND_JOINT_IDS[-1], pose[0], pose[1])
    #     joint_poses = list(map(self.ang_in_mpi_ppi, joint_poses))
    #     p.setJointMotorControlArray(self.robot_id,controlMode = p.POSITION_CONTROL, jointIndices = self._LEFT_HAND_JOINT_IDS,targetPositions  = joint_poses[9:16])

    #     return joint_poses[9:16]

    # def move_right_arm(self,pose):        
    #     joint_poses = p.calculateInverseKinematics(self.robot_id, self._RIGHT_HAND_JOINT_IDS[-1], pose[0], pose[1])
    #     joint_poses = list(map(self.ang_in_mpi_ppi, joint_poses))
    #     p.setJointMotorControlArray(self.robot_id,controlMode = p.POSITION_CONTROL, jointIndices = self._RIGHT_HAND_JOINT_IDS,targetPositions  = joint_poses[:7])

    #     return joint_poses[:7] 
        pass

    def read_joint_states_cb(self, msg):
        self.q_curr_joint = np.asarray(list(msg.position)[9:23])
        self.q_dot_curr_joint = np.asarray(list(msg.velocity)[9:23])
        self.joint_names_position = msg.name[9:23]
        self.q_curr_joint_global = self.q_curr_joint
        self.q_dot_curr_joint_global = self.q_dot_curr_joint
        # print(self.q_curr_joint)
        # print(self.joint_names_position)

    def read_base_states_cb(self, msg):
        base_euler_angle = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.q_curr_base = [msg.pose.pose.position.x, msg.pose.pose.position.y, base_euler_angle[2]]
        self.q_dot_curr_base = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z])
        self.YumiBase_position = np.asarray([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])    
        self.YumiBase_rotation = optas.spatialmath.rotz(base_euler_angle[2])
        self.q_curr_base_global = self.q_curr_base
        self.q_dot_curr_base_global = self.q_dot_curr_base
        # print(self.q_curr_base)

    def polynomial_trajectory_yumi_base(self, des_pos):

        pol_traj_result_Px = []
        pol_traj_result_Py = []
        pol_traj_result_Rz = []

        self.start_pos = self.q_curr_base_global
        self.end_pos = [des_pos[0], des_pos[1], des_pos[2]]
        self.total_time = 10 

        self.start_time = 0
        self.end_time = self.total_time
        self.num_steps = 10

        self.time_vector = np.linspace(self.start_time, self.end_time, self.num_steps)

        # Calculate coefficients of 5th order polynomial
        self.a0_Px = self.start_pos[0]
        self.a1_Px = 0
        self.a2_Px = 0
        self.a3_Px = 10*(self.end_pos[0] - self.start_pos[0])/(self.total_time**3)
        self.a4_Px = -15*(self.end_pos[0] - self.start_pos[0])/(self.total_time**4)
        self.a5_Px = 6*(self.end_pos[0] - self.start_pos[0])/(self.total_time**5)

        self.a0_Py = self.start_pos[1]
        self.a1_Py = 0
        self.a2_Py = 0
        self.a3_Py = 10*(self.end_pos[1] - self.start_pos[1])/(self.total_time**3)
        self.a4_Py = -15*(self.end_pos[1] - self.start_pos[1])/(self.total_time**4)
        self.a5_Py = 6*(self.end_pos[1] - self.start_pos[1])/(self.total_time**5)

        self.a0_Rz = self.start_pos[2]
        self.a1_Rz = 0
        self.a2_Rz = 0
        self.a3_Rz = 10*(self.end_pos[2] - self.start_pos[2])/(self.total_time**3)
        self.a4_Rz = -15*(self.end_pos[2] - self.start_pos[2])/(self.total_time**4)
        self.a5_Rz = 6*(self.end_pos[2] - self.start_pos[2])/(self.total_time**5)

        for i in range(0, len(self.time_vector)):
            pos_x = self.a0_Px + self.a1_Px*self.time_vector[i] + self.a2_Px*(self.time_vector[i]**2) + self.a3_Px*(self.time_vector[i]**3) + self.a4_Px*(self.time_vector[i]**4) + self.a5_Px*(self.time_vector[i]**5)
            pos_y = self.a0_Py + self.a1_Py*self.time_vector[i] + self.a2_Py*(self.time_vector[i]**2) + self.a3_Py*(self.time_vector[i]**3) + self.a4_Py*(self.time_vector[i]**4) + self.a5_Py*(self.time_vector[i]**5)
            rot_z = self.a0_Rz + self.a1_Rz*self.time_vector[i] + self.a2_Rz*(self.time_vector[i]**2) + self.a3_Rz*(self.time_vector[i]**3) + self.a4_Rz*(self.time_vector[i]**4) + self.a5_Rz*(self.time_vector[i]**5)

            pol_traj_result_Px.append(pos_x)
            pol_traj_result_Py.append(pos_y)
            pol_traj_result_Rz.append(rot_z)

        pol_traj_result = [pol_traj_result_Px, pol_traj_result_Py, pol_traj_result_Rz]    

        return pol_traj_result 

    def polynomial_trajectory_yumi_end_eff(self, init_pos, des_pos, pol_traj_total_global_time, pol_traj_global_num_steps):

        gt = rospy.get_time()

        pol_traj_result_Px = []
        pol_traj_result_Py = []
        pol_traj_result_Pz = []

        self.start_pos = [init_pos[0], init_pos[1], init_pos[2]]
        self.end_pos = [des_pos[0], des_pos[1], des_pos[2]]
        self.total_time = pol_traj_total_global_time

        self.start_time = 0
        self.end_time = self.total_time
        self.num_steps = pol_traj_global_num_steps

        self.time_vector = np.linspace(self.start_time, self.end_time, self.num_steps)

        # Calculate coefficients of 5th order polynomial
        self.a0_Px = self.start_pos[0]
        self.a1_Px = 0
        self.a2_Px = 0
        self.a3_Px = 10*(self.end_pos[0] - self.start_pos[0])/(self.total_time**3)
        self.a4_Px = -15*(self.end_pos[0] - self.start_pos[0])/(self.total_time**4)
        self.a5_Px = 6*(self.end_pos[0] - self.start_pos[0])/(self.total_time**5)

        self.a0_Py = self.start_pos[1]
        self.a1_Py = 0
        self.a2_Py = 0
        self.a3_Py = 10*(self.end_pos[1] - self.start_pos[1])/(self.total_time**3)
        self.a4_Py = -15*(self.end_pos[1] - self.start_pos[1])/(self.total_time**4)
        self.a5_Py = 6*(self.end_pos[1] - self.start_pos[1])/(self.total_time**5)

        self.a0_Pz = self.start_pos[2]
        self.a1_Pz = 0
        self.a2_Pz = 0
        self.a3_Pz = 10*(self.end_pos[2] - self.start_pos[2])/(self.total_time**3)
        self.a4_Pz = -15*(self.end_pos[2] - self.start_pos[2])/(self.total_time**4)
        self.a5_Pz = 6*(self.end_pos[2] - self.start_pos[2])/(self.total_time**5)

        for i in range(0, len(self.time_vector)):
            pos_x = self.a0_Px + self.a1_Px*self.time_vector[i] + self.a2_Px*(self.time_vector[i]**2) + self.a3_Px*(self.time_vector[i]**3) + self.a4_Px*(self.time_vector[i]**4) + self.a5_Px*(self.time_vector[i]**5)
            pos_y = self.a0_Py + self.a1_Py*self.time_vector[i] + self.a2_Py*(self.time_vector[i]**2) + self.a3_Py*(self.time_vector[i]**3) + self.a4_Py*(self.time_vector[i]**4) + self.a5_Py*(self.time_vector[i]**5)
            pos_z = self.a0_Pz + self.a1_Pz*self.time_vector[i] + self.a2_Pz*(self.time_vector[i]**2) + self.a3_Pz*(self.time_vector[i]**3) + self.a4_Pz*(self.time_vector[i]**4) + self.a5_Pz*(self.time_vector[i]**5)

            pol_traj_result_Px.append(pos_x)
            pol_traj_result_Py.append(pos_y)
            pol_traj_result_Pz.append(pos_z)

        pol_traj_result = [pol_traj_result_Px, pol_traj_result_Py, pol_traj_result_Pz]    

        return pol_traj_result 

    def yumi_mobile_base_control(self, desired_base_motion_pos, vec_count):

        gt = rospy.get_time()
        # Concatanation of all the robot joints
        self.q_curr_whole_body = np.concatenate((self.q_curr_base_global, self.q_curr_joint_global), axis=None)
        self.q_curr_whole_body_joint_names = self.joint_names_base + self.yumi_robot_actuated_joint_names[9:23]

        # Yumi Base Motion Control
        # for i in range (0, duration):
        self.q_des_base_global[0] = desired_base_motion_pos[0][vec_count]
        self.q_des_base_global[1] = desired_base_motion_pos[1][vec_count]
        self.q_des_base_global[2] = desired_base_motion_pos[2][vec_count]
        # Defining the position-orientation error of yumi mobile base
        self.q_tilde_base_global[0] = self.q_des_base_global[0] - self.q_curr_base_global[0]
        self.q_tilde_base_global[1] = self.q_des_base_global[1] - self.q_curr_base_global[1]
        self.q_tilde_base_global[2] = self.q_des_base_global[2] - self.q_curr_base_global[2]
        # Defining the variable stiffness profile
        self.K_Px_total = self.K_Px_const + 1.75 * np.abs(self.q_tilde_base_global[0])
        self.K_Py_total = self.K_Py_const + 1.75 * np.abs(self.q_tilde_base_global[1])
        self.K_Rz_total = self.K_Rz_const + 1.75 * np.abs(self.q_tilde_base_global[2])
        # Defining the variable damping profile
        self.D_Px_total = self.D_Px_const # +  0.5 * np.abs(self.q_tilde_base_global[0])/gt
        self.D_Py_total = self.D_Py_const # +  0.5 * np.abs(self.q_tilde_base_global[1])/gt
        self.D_Rz_total = self.D_Rz_const # +  0.5 * np.abs(self.q_tilde_base_global[2])/gt
        # Moving the robot base to its desired location - variable velocity
        self.yumi_base_msg.linear.x  = self.K_Px_total * self.q_tilde_base_global[0]/gt - self.D_Px_total * self.q_curr_base_global[0]/gt  # set linear velocity in x direction
        self.yumi_base_msg.linear.y  = self.K_Py_total * self.q_tilde_base_global[1]/gt - self.D_Py_total * self.q_curr_base_global[1]/gt  # set linear velocity in y direction
        self.yumi_base_msg.angular.z = self.K_Rz_total * self.q_tilde_base_global[2]/gt - self.D_Rz_total * self.q_curr_base_global[2]/gt  # set angular velocity in z direction  

        # Sending motion commands to yumi mobile base
        self.joint_pub_yumi_base_velocity.publish(self.yumi_base_msg)

        print(f"{self.q_curr_base_global[0]} --- {self.q_curr_base_global[1]} --- {self.q_curr_base_global[2]}")
        # print(f"{self.q_tilde_base_global[0]} --- {self.q_tilde_base_global[1]} --- {self.q_tilde_base_global[2]}")
        # print(vec_count)

    def yumi_dual_arm_control(self, init_left_end_pos, des_left_end_pos, init_right_end_pos, des_right_end_pos, vec_count):

        ori_left = p.getQuaternionFromEuler([np.pi/2,0,0])
        desired_left_end_eff_motion_pos = yumi_robot_control.polynomial_trajectory_yumi_end_eff(init_left_end_pos, des_left_end_pos)
        ori_right = p.getQuaternionFromEuler([-np.pi/2,0,0])
        desired_right_end_eff_motion_pos = yumi_robot_control.polynomial_trajectory_yumi_end_eff(init_right_end_pos, des_right_end_pos)
        
        # for i in range (0, duration):
        pos_left_pol_traj = [desired_left_end_eff_motion_pos[0][vec_count], desired_left_end_eff_motion_pos[1][vec_count], desired_left_end_eff_motion_pos[2][vec_count]]
        pose_left = [pos_left_pol_traj, ori_left]
        pos_right_pol_traj = [desired_right_end_eff_motion_pos[0][vec_count], desired_right_end_eff_motion_pos[1][vec_count], desired_right_end_eff_motion_pos[2][vec_count]]
        pose_right = [pos_right_pol_traj, ori_right]

        yumi_left_arm_pos = yumi_robot_control.move_left_arm(pose = pose_left)
        yumi_right_arm_pos = yumi_robot_control.move_right_arm(pose = pose_right)
        
        yumi_robot_control.publish_robot_arm_joints_cmd(yumi_left_arm_pos, yumi_right_arm_pos)

    def publish_robot_arm_joints_cmd(self, yumi_left_arm_pos, yumi_right_arm_pos):

        self.yumi_left_arm_pos = yumi_left_arm_pos
        self.yumi_right_arm_pos = yumi_right_arm_pos

        # Sending motion commands to yumi left arm
        self.yumi_robl_arm_joint_pub1.publish(self.yumi_left_arm_pos[0])
        self.yumi_robl_arm_joint_pub2.publish(self.yumi_left_arm_pos[1])
        self.yumi_robl_arm_joint_pub3.publish(self.yumi_left_arm_pos[2])
        self.yumi_robl_arm_joint_pub4.publish(self.yumi_left_arm_pos[3])
        self.yumi_robl_arm_joint_pub5.publish(self.yumi_left_arm_pos[4])
        self.yumi_robl_arm_joint_pub6.publish(self.yumi_left_arm_pos[5])
        self.yumi_robl_arm_joint_pub7.publish(self.yumi_left_arm_pos[6])

        # Sending motion commands to yumi right arm
        self.yumi_robr_arm_joint_pub1.publish(self.yumi_right_arm_pos[0])
        self.yumi_robr_arm_joint_pub2.publish(self.yumi_right_arm_pos[1])
        self.yumi_robr_arm_joint_pub3.publish(self.yumi_right_arm_pos[2])
        self.yumi_robr_arm_joint_pub4.publish(self.yumi_right_arm_pos[3])
        self.yumi_robr_arm_joint_pub5.publish(self.yumi_right_arm_pos[4])
        self.yumi_robr_arm_joint_pub6.publish(self.yumi_right_arm_pos[5])
        self.yumi_robr_arm_joint_pub7.publish(self.yumi_right_arm_pos[6])  

        # Sending motion commands to yumi mobile base
        self.joint_pub_yumi_base_velocity.publish(self.yumi_base_msg)

    def dual_arm_initial_configuration(self):
        self.yumi_robl_arm_joint_pub1.publish(self.YUMI_LEFT_ARM_HOME_POSITION[0])
        self.yumi_robl_arm_joint_pub2.publish(self.YUMI_LEFT_ARM_HOME_POSITION[1])
        self.yumi_robl_arm_joint_pub3.publish(self.YUMI_LEFT_ARM_HOME_POSITION[2])
        self.yumi_robl_arm_joint_pub4.publish(self.YUMI_LEFT_ARM_HOME_POSITION[3])
        self.yumi_robl_arm_joint_pub5.publish(self.YUMI_LEFT_ARM_HOME_POSITION[4])
        self.yumi_robl_arm_joint_pub6.publish(self.YUMI_LEFT_ARM_HOME_POSITION[5])
        self.yumi_robl_arm_joint_pub7.publish(self.YUMI_LEFT_ARM_HOME_POSITION[6])
    
        self.yumi_robr_arm_joint_pub1.publish(self.YUMI_RIGHT_ARM_HOME_POSITION[0])
        self.yumi_robr_arm_joint_pub2.publish(self.YUMI_RIGHT_ARM_HOME_POSITION[1])
        self.yumi_robr_arm_joint_pub3.publish(self.YUMI_RIGHT_ARM_HOME_POSITION[2])
        self.yumi_robr_arm_joint_pub4.publish(self.YUMI_RIGHT_ARM_HOME_POSITION[3])
        self.yumi_robr_arm_joint_pub5.publish(self.YUMI_RIGHT_ARM_HOME_POSITION[4])
        self.yumi_robr_arm_joint_pub6.publish(self.YUMI_RIGHT_ARM_HOME_POSITION[5])
        self.yumi_robr_arm_joint_pub7.publish(self.YUMI_RIGHT_ARM_HOME_POSITION[6])

    def yumi_base_initial_pose(self):
        gt = rospy.get_time()

        self.yumi_base_msg.linear.x  = self.K_Px_const * (self.q_des_base_global[0] - self.q_curr_base_global[0])/gt  # set linear velocity in x direction
        self.yumi_base_msg.linear.y  = self.K_Py_const * (self.q_des_base_global[1] - self.q_curr_base_global[1])/gt    # set linear velocity in y direction
        self.yumi_base_msg.angular.z = self.K_Rz_const * (self.q_des_base_global[2] - self.q_curr_base_global[2])/gt    # set angular velocity in z direction   

        self.joint_pub_yumi_base_velocity.publish(self.yumi_base_msg)
             
    def desired_wholebody_pos_command(self, des_pos_YumiBase, des_ori_YumiBase, des_pos_YumiR, des_ori_YumiR, des_pos_YumiL, des_ori_YumiL, des_duration):

        # Initializing node class
        YumiBase_position = des_pos_YumiBase
        YumiBase_angle = des_ori_YumiBase # float(yumi_robot_control.q_curr_base_global[2])
        YumiBase_rot_rpy = [0.0, 0.0, YumiBase_angle]
        YumiBase_quat = optas.spatialmath.Quaternion.fromrpy(YumiBase_rot_rpy).getquat()

        right_arm_position_default = des_pos_YumiR
        right_arm_orientation_default = des_ori_YumiR

        left_arm_position_default = des_pos_YumiL
        left_arm_orientation_default = des_ori_YumiL

        # Parse arguments from terminal
        parser = argparse.ArgumentParser(description='Client node to command robot base and end-effector pose.')

        # Parse YumiBase arguments
        parser.add_argument('--target_position_YumiBase', nargs=3,
            help="Give target position of the robot in meters.",
            type=float, default=YumiBase_position,
            metavar=('POS_X', 'POS_Y', 'POS_Z')
        )

        YumiBase_quaternion = optas.spatialmath.Quaternion(YumiBase_quat[0], YumiBase_quat[1], YumiBase_quat[2], YumiBase_quat[3])
        parser.add_argument('--target_orientation_YumiBase', nargs=4,
            help="Give target orientation as a quaternion.",
            type=float, default=[YumiBase_quat[0], YumiBase_quat[1], YumiBase_quat[2], YumiBase_quat[3]],
            metavar=('QUAT_X','QUAT_Y','QUAT_Z','QUAT_W')
        )

        # Parse right arm arguments
        right_arm_position_after_rotation = optas.spatialmath.rotz(YumiBase_angle) @ right_arm_position_default.transpose() + np.asarray(YumiBase_position)
        parser.add_argument('--target_position_YumiR', nargs=3,
            help="Give target position of the robot in meters.",
            type=float, default=[right_arm_position_after_rotation[0], right_arm_position_after_rotation[1], right_arm_position_after_rotation[2]],
            metavar=('POS_X', 'POS_Y', 'POS_Z')
        )

        right_arm_orientation_quaternion = right_arm_orientation_default.__mul__(YumiBase_quaternion)
        right_arm_quaternion = [right_arm_orientation_quaternion.split()[0], right_arm_orientation_quaternion.split()[1], right_arm_orientation_quaternion.split()[2], right_arm_orientation_quaternion.split()[3]]
        parser.add_argument('--target_orientation_YumiR', nargs=4,
            help="Give target orientation as a quaternion.",
            type=float, default=right_arm_quaternion,
            metavar=('QUAT_X','QUAT_Y','QUAT_Z','QUAT_W')
        )

        # Parse left arm arguments
        left_arm_position_after_rotation = optas.spatialmath.rotz(YumiBase_angle) @ left_arm_position_default.transpose() + np.asarray(YumiBase_position)
        parser.add_argument('--target_position_YumiL', nargs=3,
            help="Give target position of the robot in meters.",
            type=float, default=[left_arm_position_after_rotation[0], left_arm_position_after_rotation[1], left_arm_position_after_rotation[2]],
            metavar=('POS_X', 'POS_Y', 'POS_Z')
        )

        left_arm_orientation_quaternion = left_arm_orientation_default.__mul__(YumiBase_quaternion)
        left_arm_quaternion = [left_arm_orientation_quaternion.split()[0], left_arm_orientation_quaternion.split()[1], left_arm_orientation_quaternion.split()[2], left_arm_orientation_quaternion.split()[3]]
        parser.add_argument('--target_orientation_YumiL', nargs=4,
            help="Give target orientation as a quaternion.",
            type=float, default=left_arm_quaternion,
            metavar=('QUAT_X','QUAT_Y','QUAT_Z','QUAT_W')
        )
        parser.add_argument("--duration", help="Give duration of motion in seconds.", type=float, default = float(des_duration))
        args = vars(parser.parse_args())

        cmd_yumi_pose_client = CmdYumiPoseClient(rospy.get_name(),
            args['target_position_YumiBase'],
            args['target_orientation_YumiBase'],
            args['target_position_YumiR'],
            args['target_orientation_YumiR'],
            args['target_position_YumiL'],
            args['target_orientation_YumiL'],
            args['duration']
        ) 

        return cmd_yumi_pose_client

    def whole_body_optimal_control(self, target_pos_YumiBase, target_quat_YumiBase, target_pos_YumiR, target_quat_YumiR, target_pos_YumiL, target_quat_YumiL, duration):
        
        # Desired end-effector position and orientation
        pos_Left = [
            float(target_pos_YumiL[0]),
            float(target_pos_YumiL[1]),
            float(target_pos_YumiL[2])
        ]    
        pos_Right = [
            float(target_pos_YumiR[0]),
            float(target_pos_YumiR[1]),
            float(target_pos_YumiR[2])
        ]
        ori_Left = [
            float(target_quat_YumiL[0]),
            float(target_quat_YumiL[1]),
            float(target_quat_YumiL[2]),
            float(target_quat_YumiL[3])
        ]        
        ori_Right = [
            float(target_quat_YumiR[0]),
            float(target_quat_YumiR[1]),
            float(target_quat_YumiR[2]),
            float(target_quat_YumiR[3])
        ]

        # Setting initial seed
        self.solver_yumi_robot.reset_initial_seed({f'{self.yumi_robot_name}/q': self.q_nom[self.yumi_robot_opt_idx]})
        self.solver_yumi_robot.reset_parameters({'pos_R': pos_Right, 'ori_R': ori_Right, 'pos_L': pos_Left, 'ori_L': ori_Left, f'{self.yumi_robot_name}/P': self.q_nom[self.yumi_robot_param_idx]})

        # Initialize variables for 2 7-DoF arms
        self.q_curr_joint = np.zeros(self.yumi_robot_ndof - 11)
        self.q_curr_base = np.zeros(self.yumi_ndof_base)
        self.joint_names_base = ['yumi_base_joint_p_x', 'yumi_base_joint_p_y', 'yumi_base_joint_r_z']
        qT = np.zeros(yumi_robot_control.yumi_robot_ndof)


        # Solve problem
        solution = self.solver_yumi_robot.solve()
        self.qT_array = solution[f'{self.yumi_robot_name}/q']
        
        # Saving solution
        # self.qT_array[self.yumi_robot_opt_idx] = np.asarray(self.qT_array).T[0]
        # self.qT_array[self.yumi_robot_param_idx] = np.asarray(self.q_nom[self.yumi_robot_param_idx]).T[0]

        print(f'\nOptimised solution (25-DoF) = \n{self.qT_array}')

if __name__=="__main__":

    rospy.init_node("yumi_robot_control", anonymous=True)

    rate = rospy.Rate(100) # 100hz
    gt0 = rospy.Time.now()

    # Initializing node class
    yumi_robot_control = CmdYumiRobotControl(rospy.get_name())

    yumi_robot_control.pol_traj_global_duration_time = 10
    yumi_robot_control.pol_traj_global_num_steps = 10

    # End-effector(s) Static Targets
    des_pos_YumiBase = np.array([yumi_robot_control.q_curr_base_global[0], yumi_robot_control.q_curr_base_global[1], 0.0])
    des_ori_YumiBase = yumi_robot_control.q_curr_base_global[2]

    des_pos_YumiL =  np.array([0.5, 0.35, 1.0])
    des_ori_YumiL_rpy = [0.0, 90.0, 0.0]
    des_ori_YumiL =  optas.spatialmath.Quaternion.fromrpy(des_ori_YumiL_rpy)

    des_pos_YumiR =  np.array([0.5, -0.35, 1.0])
    des_ori_YumiR_rpy = [0.0, 90.0, 0.0]
    des_ori_YumiR =  optas.spatialmath.Quaternion.fromrpy(des_ori_YumiR_rpy)

    des_duration = yumi_robot_control.pol_traj_global_duration_time 

    cmd_pos_client = yumi_robot_control.desired_wholebody_pos_command(des_pos_YumiBase, des_ori_YumiBase, des_pos_YumiR, des_ori_YumiR, des_pos_YumiL, des_ori_YumiL, des_duration)
    yumi_robot_control.whole_body_optimal_control(cmd_pos_client.target_pos_YumiBase, cmd_pos_client.target_quat_YumiBase,
                                                  cmd_pos_client.target_pos_YumiR, cmd_pos_client.target_quat_YumiR, 
                                                  cmd_pos_client.target_pos_YumiL, cmd_pos_client.target_quat_YumiL, cmd_pos_client.duration)
    
    # End-effector(s) Polynomial Trajectory - OPTAS 
    init_left_end_pos = des_pos_YumiL
    des_left_end_pos = [des_pos_YumiL[0] + 0.1, des_pos_YumiL[1] - 0.15, des_pos_YumiL[2] + 0.1]

    init_right_end_pos = des_pos_YumiR
    des_right_end_pos = [des_pos_YumiR[0] + 0.1, des_pos_YumiR[1] + 0.15, des_pos_YumiR[2] + 0.1]

    desired_left_end_eff_motion_pos = yumi_robot_control.polynomial_trajectory_yumi_end_eff(init_left_end_pos, des_left_end_pos, yumi_robot_control.pol_traj_global_duration_time, yumi_robot_control.pol_traj_global_num_steps)
    desired_right_end_eff_motion_pos = yumi_robot_control.polynomial_trajectory_yumi_end_eff(init_right_end_pos, des_right_end_pos, yumi_robot_control.pol_traj_global_duration_time, yumi_robot_control.pol_traj_global_num_steps)

    duration = len(desired_left_end_eff_motion_pos[0])

    for i in range(0, duration):
        des_pos_YumiL_pol_traj = np.array([desired_left_end_eff_motion_pos[0][i], desired_left_end_eff_motion_pos[1][i], desired_left_end_eff_motion_pos[2][i]])
        des_pos_YumiR_pol_traj = np.array([desired_right_end_eff_motion_pos[0][i], desired_right_end_eff_motion_pos[1][i], desired_right_end_eff_motion_pos[2][i]])

        cmd_pos_client_pol_traj = yumi_robot_control.desired_wholebody_pos_command(des_pos_YumiBase, des_ori_YumiBase, des_pos_YumiR_pol_traj, des_ori_YumiR, des_pos_YumiL_pol_traj, des_ori_YumiL, des_duration)
        yumi_robot_control.whole_body_optimal_control(cmd_pos_client_pol_traj.target_pos_YumiBase, cmd_pos_client_pol_traj.target_quat_YumiBase,
                                                  cmd_pos_client_pol_traj.target_pos_YumiR, cmd_pos_client_pol_traj.target_quat_YumiR, 
                                                  cmd_pos_client_pol_traj.target_pos_YumiL, cmd_pos_client_pol_traj.target_quat_YumiL, cmd_pos_client_pol_traj.duration)
        
        yumi_left_arm_pos = yumi_robot_control.qT_array[9:16]
        yumi_right_arm_pos = yumi_robot_control.qT_array[16:23]

        yumi_robot_control.publish_robot_arm_joints_cmd(yumi_left_arm_pos, yumi_right_arm_pos)

        yumi_robot_control.qT_array_pol_traj_list.append(yumi_robot_control.qT_array)    

    print("\n Optimised solution - robot joints - list (25-DoF): \n")
    for i in range(0, len(yumi_robot_control.qT_array_pol_traj_list)):
        print(f"{yumi_robot_control.qT_array_pol_traj_list[i]}")

    print(len(yumi_robot_control.qT_array_pol_traj_list))    

    while not rospy.is_shutdown():
        
        gt = rospy.Time.now() - gt0
        
        if gt.secs < 10000:

            # yumi_robot_control.dual_arm_initial_configuration()
            yumi_robot_control.yumi_base_initial_pose()
            
            yumi_left_arm_pos = yumi_robot_control.qT_array[9:16]
            yumi_right_arm_pos = yumi_robot_control.qT_array[16:23]

            yumi_robot_control.publish_robot_arm_joints_cmd(yumi_left_arm_pos, yumi_right_arm_pos)

            (end_eff_left_pos, end_eff_left_ori) = yumi_robot_control.yumi_robl_arm_end_eff.lookupTransform('/yumi_robl_link_7', '/base_link', rospy.Time(0))
            (end_eff_right_pos, end_eff_right_ori) = yumi_robot_control.yumi_robr_arm_end_eff.lookupTransform('/yumi_robr_link_7', '/base_link', rospy.Time(0))
            print(end_eff_left_pos, end_eff_right_pos)


            # yumi_robot_control.vec_count = (gt - yumi_robot_control.duration_coef * duration) % duration
            # yumi_robot_control.vec_count = int(yumi_robot_control.vec_count)

            # if yumi_robot_control.vec_count < duration:
            #     yumi_robot_control.duration_coef = yumi_robot_control.duration_coef
            # elif yumi_robot_control.vec_count >= duration:
            #     yumi_robot_control.duration_coef += 1

            # if gt > 20 and gt < 30:
            #     yumi_robot_control.vec_count = yumi_robot_control.vec_count

            #     yumi_left_arm_pos = yumi_robot_control.qT_array_pol_traj_list[yumi_robot_control.vec_count][9:16]
            #     yumi_right_arm_pos = yumi_robot_control.qT_array_pol_traj_list[yumi_robot_control.vec_count][16:23]

            #     yumi_robot_control.publish_robot_arm_joints_cmd(yumi_left_arm_pos, yumi_right_arm_pos)

            # elif gt > 30 and gt < 1000:
            #     yumi_robot_control.vec_count = duration - 1

            #     yumi_left_arm_pos = yumi_robot_control.qT_array_pol_traj_list[yumi_robot_control.vec_count][9:16]
            #     yumi_right_arm_pos = yumi_robot_control.qT_array_pol_traj_list[yumi_robot_control.vec_count][16:23]

            #     yumi_robot_control.publish_robot_arm_joints_cmd(yumi_left_arm_pos, yumi_right_arm_pos)

        else:
            
            des_pos_base = [0 * 4.0, 0 * 2.70, 0.0]
            desired_base_motion_pos = yumi_robot_control.polynomial_trajectory_yumi_base(des_pos_base)
            duration = len(desired_base_motion_pos[0])

            yumi_robot_control.vec_count = (gt - yumi_robot_control.duration_coef * duration) % duration
            yumi_robot_control.vec_count = int(yumi_robot_control.vec_count)

            if yumi_robot_control.vec_count < duration:
                yumi_robot_control.duration_coef = yumi_robot_control.duration_coef
            elif yumi_robot_control.vec_count >= duration:
                yumi_robot_control.duration_coef += 1

            yumi_robot_control.yumi_mobile_base_control(desired_base_motion_pos, yumi_robot_control.vec_count)
            yumi_robot_control.publish_robot_arm_joints_cmd(yumi_left_arm_pos, yumi_right_arm_pos)

            # des_pos_YumiBase = np.array([yumi_robot_control.q_curr_base_global[0], yumi_robot_control.q_curr_base_global[1], 0.0])
            # des_ori_YumiBase = yumi_robot_control.q_curr_base_global[2]
            # des_pos_YumiR =  np.array([0.5, -0.35, 1.0])
            # des_ori_YumiR_rpy = [0.0, 90.0, 0.0]
            # des_ori_YumiR =  optas.spatialmath.Quaternion.fromrpy(des_ori_YumiR_rpy)
            # des_pos_YumiL =  np.array([0.5, 0.35, 1.0])
            # des_ori_YumiL_rpy = [0.0, 90.0, 0.0]
            # des_ori_YumiL =  optas.spatialmath.Quaternion.fromrpy(des_ori_YumiL_rpy)
            # des_duration = 5.0

            # cmd_pos_client = yumi_robot_control.desired_wholebody_pos_command(des_pos_YumiBase, des_ori_YumiBase, des_pos_YumiR, des_ori_YumiR, des_pos_YumiL, des_ori_YumiL, des_duration)
            # yumi_robot_control.whole_body_optimal_control(cmd_pos_client.target_pos_YumiBase, cmd_pos_client.target_quat_YumiBase, 
            #                                               cmd_pos_client.target_pos_YumiR, cmd_pos_client.target_quat_YumiR, 
            #                                               cmd_pos_client.target_pos_YumiL, cmd_pos_client.target_quat_YumiL, cmd_pos_client.duration)
            
            # yumi_left_arm_pos = yumi_robot_control.qT_array[9:16]
            # yumi_right_arm_pos = yumi_robot_control.qT_array[16:23]

            # yumi_robot_control.publish_robot_arm_joints_cmd(yumi_left_arm_pos, yumi_right_arm_pos)

            # print(cmd_pos_client.target_pos_YumiBase)
            # print(cmd_pos_client.target_pos_YumiR)