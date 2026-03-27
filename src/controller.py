#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Upper body base controller class
import numpy as np
import os
import rospy
from typing import Dict, Union, List, Tuple
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Image
from kuavo_msgs.msg import robotHeadMotionData, dexhandCommand, twoArmHandPoseCmd, ikSolveParam, dexhandTouchState
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse, twoArmHandPoseCmdSrv
import sys
import threading
import cv2
from datetime import datetime
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
# sys.path.insert(0, '/home/lab/kuavo-ros-opensource/src/my_controller')
# from my_arm_move import my_arm_continuous_move, my_arm_to_origin
from termcolor import cprint



class Controller():
    '''
    Kuavo base controller that enable uppper body control of kuavo 4pro maxB
    Left arm: 7-DoF
    Right arm: 7-DoF
    Head: 2-DoF
    Left dex hand: 6-DoF
    Right dex hand: 6-DoF
    '''

    def __init__(self, config:dict):
        self.config = config
        self.use_camera = False

        # proprioception state
        self._current_head_joint_pos = None # head yaw, head pitch
        self._current_arm_joint_pos = None # left arm 7 + right 7
        self._current_dexhand_joint_pos = None # left hand 6 + right hand 6
        # vision state
        self.head_color_img = None
        self.head_depth_img = None
        self.left_hand_wrist_color_img = None
        self.left_hand_wrist_depth_img = None
        self.right_hand_wrist_color_img = None
        self.right_hand_wrist_depth_img = None
        # tactile state
        self.tactile_state = None
        self.left_hand_tactile_state = np.zeros((5, 12), dtype=np.float64)
        self.right_hand_tactile_state = np.zeros((5, 12), dtype=np.float64)
        self.left_hand_contact = 0
        self.right_hand_contact = 0
        self._contact_threshold_n = 1.0 # threshold for contact detection based on normal force sum of thumb+index fingers, in [N]

        # predefined pose
        self.ges_hand_init = [0, 0, 0, 0, 0, 0]
        self.ges_hand_grab = [60, 100, 60, 60, 60, 60]
        self.ges_hand_release = [0, 100, 0, 0, 0, 0]

        self.ges_hand_pinch = [60, 100, 60, 60, 0, 0]
        self.ges_arm_prepare = np.array([-10, 20, -30, -100, -30, 0, 0,    -10, -20, 30, -100, 30, 0, 0], dtype=np.float64)
        self.ges_arm_left_twist_away = np.array([-10, 20, 50, -90, -30, 0, 0,    -10, -20, 30, -100, 30, 0, 0], dtype=np.float64)
        self.ges_arm_right_twist_away = np.array([-10, 20, -30, -100, -30, 0, 0,    -10, -20, -50, -90, 30, 0, 0], dtype=np.float64)
        # stand by zero pose.
        self.init_pos = np.array([  0, 0, 0, 0, 0, 0, 0,                       # left
                                    0, 0, 0, 0, 0, 0, 0], dtype=np.float64)    # right
        self.intermediate_pos = np.array([  65, 45, 0, -120, 0, 0, 0,                  
                                            65, -45, 0, -120, 0, 0, 0], dtype=np.float64)
        # self.intermediate_pos = np.array([  20, 65, 0, 0, 0, 0, 0,                  
        #                                     20, -65, 0, 0, 0, 0, 0], dtype=np.float64)
        self.prepare_pos = np.array([   0, 0, 0, -90, 0, 0, 0,    
                                        0, 0, 0, -90, 0, 0, 0], dtype=np.float64)

        self.prepared_flag = False

        # ROS init node
        rospy.init_node('kuavo_upper_body_base_controller')
        # change to external control mode
        self._arm_traj_change_mode_client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)
        request = changeArmCtrlModeRequest()
        request.control_mode = 2  # 设置控制模式: 0 keep pose, 1: auto_swing_arm, 2 external_control
        self._arm_traj_change_mode_client(request)
        # publishers
        # head
        self.head_pose_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
        # arms
        # TODO: check eef control mode or joint space control 
        self.arm_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
        # dex-hands
        self.hand_pub = rospy.Publisher('/dexhand/command', dexhandCommand, queue_size=10)
        
        # proprio state subscribers
        # /humanoid_controller/optimizedState_mrt/joint_pos (Float64MultiArray):
        #   data = full-body joint positions in radians; callback takes the last 14 entries as L/R arm joints
        #   and converts rad -> deg (x 57.3) for convenience.
        self.arm_joint_sub =  rospy.Subscriber('/humanoid_controller/optimizedState_mrt/joint_pos', Float64MultiArray, self._arm_joint_pos_callback)
        # /robot_head_motion_data (robotHeadMotionData):
        #   joint_data[0] = yaw (deg), joint_data[1] = pitch (deg)
        self.head_joint_sub = rospy.Subscriber('/robot_head_motion_data', robotHeadMotionData, self._head_joint_pos_callback)
        # /dexhand/state (JointState):
        #   msg.position = dex-hand joint positions; 
        # 关节位置数组, 长度为12, 前6个为左手关节位置, 后6个为右手关节位置
        #   and units should match the dexhand driver (typically degrees, same scale as /dexhand/command data).
        self.dexhand_sub = rospy.Subscriber('/dexhand/state', JointState, self._hand_joint_pos_callback)
        self.dexhand_touch_sub = rospy.Subscriber('/dexhand/touch_state', dexhandTouchState, self._hand_tactile_state_callback)
        
    
        # cameras
        if self.use_camera:
            self.head_color_cam_sub = rospy.Subscriber("/head_camera/color/image_raw", Image, 
                                                    self._head_color_image_callback)
            self.head_depth_cam_sub = rospy.Subscriber("/head_camera/depth/image_raw", Image, 
                                                    self._head_depth_image_callback)
            self.left_hand_wrist_color_cam_sub = rospy.Subscriber("/left_wrist_camera/color/image_raw", Image, 
                                                                self._left_hand_wrist_color_image_callback)
            self.left_hand_wrist_depth_cam_sub = rospy.Subscriber("/left_wrist_camera/depth/image_rect_raw", Image, 
                                                                self._left_hand_wrist_depth_image_callback)
            self.right_hand_wrist_color_cam_sub = rospy.Subscriber("/right_wrist_camera/color/image_raw", Image, 
                                                                self._right_hand_wrist_color_image_callback)
            self.right_hand_wrist_depth_cam_sub = rospy.Subscriber("/right_wrist_camera/depth/image_rect_raw", Image, 
                                                                self._right_hand_wrist_depth_image_callback)
            rospy.loginfo("Waiting for camera topics to be ready...")
            rospy.wait_for_message("/head_camera/color/image_raw", Image)
            rospy.wait_for_message("/head_camera/depth/image_raw", Image)
            rospy.wait_for_message("/left_wrist_camera/color/image_raw", Image)
            rospy.wait_for_message("/left_wrist_camera/depth/image_rect_raw", Image)
            rospy.wait_for_message("/right_wrist_camera/color/image_raw", Image)
            rospy.wait_for_message("/right_wrist_camera/depth/image_rect_raw", Image)
            rospy.loginfo("Camera topics are ready.")
        
        
        self.hz = 10
        self._rate = rospy.Rate(self.hz) # 10hz
        self._rate.sleep()
        # self._thread = threading.Thread(target=self._thread_job)
        # self._thread.start()
        rospy.loginfo("Kuavo base controller initialized.")
        
        
    # ============== Reset & prepare ===============
    def reset(self, initermediate:bool=True):
        '''Reset to robot initial pose'''
        self.set_head_target_pos([0, 0]) # head down
        self.set_both_hands_action(self.ges_hand_init + self.ges_hand_init)
        '''Reset to init zero pose'''
        # action_list = [self.prepare_pos, self.intermediate_pos, self.init_pos]
        if initermediate:
            action_list = [self.intermediate_pos, self.init_pos]
            self.set_both_arm_traj(action_list[0])
            self.set_both_arm_traj(action_list[1])
        else:
            self.set_both_arm_traj(self.init_pos)
        rospy.loginfo('reset to init pose')
        self._rate.sleep()
        return True


    def prepare(self):
        '''from init pose go to prepare pose'''
        self.set_head_target_pos([0, 20]) # head down
        self.set_both_hands_action(self.ges_hand_release + self.ges_hand_release)
        '''go to prepare pose for table-top manipulation'''
        # action_list = [self.init_pos, self.intermediate_pos, self.prepare_pos]
        action_list = [self.intermediate_pos, self.prepare_pos]
        self.set_both_arm_traj(action_list[0])
        self.set_both_arm_traj(action_list[1])
        rospy.loginfo('prepare for manipulation')
        self._rate.sleep()
        return True


    # Head 
    def set_head_target_pos(self, head_target:np.ndarray):
        '''
        设置头部目标位置，并发布消息 
        head_target = [yaw, pitch]
        :param yaw: 头部旋转，范围为[-30, 30]度, max [-90, 90], +left, -right
        :param pitch: 头部俯仰，范围为[-25, 25]度, max [-30, 30], +down, -up
        '''

        assert len(head_target) == 2

        head_target_msg = robotHeadMotionData()
        head_target_msg.joint_data = list(head_target)  # 偏航角和俯仰角

        self.head_pose_pub.publish(head_target_msg)

        return True

    # Arms joint control
    def set_both_arm_traj(self, target_pos:np.ndarray, duration:float=2):
        """
        控制机械臂平滑地从当前关节角度运动到目标位置。
        
        :param target_pos: 目标关节角度的numpy数组 (长度应为14,[degree])
        :param duration: 运动持续时间（秒), total step = duration * hz
        """
        assert len(target_pos) == 14

        msg = JointState()
        msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]  # 关节名称列表

        # 设置时间参数
        steps = int(duration * self.hz)  # 计算步骤数

        # 计算每一步的角度变化量
        delta_position = (target_pos - self._current_arm_joint_pos) / steps

        # 运动过程中逐步更新关节位置
        for step in range(steps):
            # 逐步更新每个关节的角度
            step_target = self._current_arm_joint_pos + delta_position
            msg.header.stamp = rospy.Time.now()  # 当前时间戳
            msg.position = step_target.tolist()  # 更新关节位置

            self.arm_pub.publish(msg)  # 发布关节状态
            self._rate.sleep()

        # 最后一次发布，确保最终目标位置被发布
        msg.position = target_pos.tolist()
        self.arm_pub.publish(msg)
    
        return True

    # 
    # def set_arm_joint_move(self, current_position, target_position, duration):
    #     """
    #     控制机械臂平滑地从当前关节角度运动到目标位置。
        
    #     :param target_position: 目标关节角度的numpy数组 (长度应为14)
    #     :param duration: 运动持续时间（秒）
    #     """
    #     msg = JointState()
    #     msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]  # 关节名称列表

    #     # 设置时间参数
    #     steps = int(duration * self.hz)  # 计算步骤数

    #     # 计算每一步的角度变化量
    #     delta_position = (target_position - current_position) / steps

    #     # 运动过程中逐步更新关节位置
    #     for step in range(steps):
    #         # 逐步更新每个关节的角度
    #         current_position += delta_position
    #         msg.header.stamp = rospy.Time.now()  # 当前时间戳
    #         msg.position = current_position.tolist()  # 更新关节位置

    #         self.arm_pub.publish(msg)  # 发布关节状态
    #         self._rate.sleep()

    #     # 最后一次发布，确保最终目标位置被发布
    #     msg.position = target_position.tolist()
    #     self.arm_pub.publish(msg)
    
    
    # Dex Hands
    def set_both_hands_action(self, hand_joint_actions, control_mode:int=0):
        '''
        Input:
            hand_joint_actions: target joint positions
        # control_mode=0 表示 POSITION_CONTROL, 1 表示 VELOCITY_CONTROL
        # data 数组长度必须 12, 前 6 控制left hand, 后 6 控制right hand
        # 位置控制时, 0 完全打开, 100 完全关闭
        # 速度控制时，-100 全开，+100 全关
        
        '''
        assert len(hand_joint_actions) == 12
        assert control_mode in [0, 1]
        
        while not rospy.is_shutdown():

            both_hand_msg = dexhandCommand()
            both_hand_msg.control_mode = 0 # TODO: currently only support pos control
            both_hand_msg.data = hand_joint_actions
            self.hand_pub.publish(both_hand_msg)

            if np.mean(self._current_dexhand_joint_pos - np.array(hand_joint_actions)) < 1.5:
                rospy.sleep(0.5)
                break
            
            self._rate.sleep()
        


    def set_eef_pos(self, left_pos, left_angle, right_pos, right_angle, left_elbow = np.zeros(3), right_elbow = np.zeros(3)):
        '''
        Input:
            left_pos: numpy array of shape (3,) for left hand end-effector position in meters
            left_angle: numpy array of shape (4,) for left hand end-effector orientation as quaternion (x, y, z, w)
            right_pos: numpy array of shape (3,) for right hand end-effector position in meters
            right_angle: numpy array of shape (4,) for right hand end-effector orientation as quaternion (x, y, z, w)
            left_elbow: numpy array of shape (3,) for left arm elbow position in meters, optional (default: zeros, not used if joint_angles_as_q0 is False)
            right_elbow: numpy array of shape (3,) for right arm elbow position in meters, optional (default: zeros, not used if joint_angles_as_q0 is False)
        '''
        # decide use custom ik param or not
        use_custom_ik_param = True
        # joint angles as initial guess for ik
        joint_angles_as_q0 = False # True for custom initial guess, False for default initial guess
        # ik solver param
        ik_solve_param = ikSolveParam()
        # snopt params
        ik_solve_param.major_optimality_tol = 1e-3
        ik_solve_param.major_feasibility_tol = 1e-3
        ik_solve_param.minor_feasibility_tol = 1e-3
        ik_solve_param.major_iterations_limit = 100
        # constraint and cost params
        ik_solve_param.oritation_constraint_tol= 1e-3
        ik_solve_param.pos_constraint_tol = 1e-3 # 0.001m, work when pos_cost_weight==0.0
        ik_solve_param.pos_cost_weight = 0.0 # If U need high accuracy, set this to 0.0 !!!
        
        eef_pose_msg = twoArmHandPoseCmd()
        eef_pose_msg.ik_param = ik_solve_param
        eef_pose_msg.use_custom_ik_param = use_custom_ik_param        # # True for custom ik param, False for default ik param  
        eef_pose_msg.joint_angles_as_q0 = joint_angles_as_q0

        # joint_angles_as_q0 为 False 时，这两个参数不会被使用
        eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7)    # rads
        eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)   # rads

        # 设置左手末端执行器的位置和姿态
        # eef_pose_msg.hand_poses.left_pose.pos_xyz =  np.array([0.3,0.18,0.11988012])
        eef_pose_msg.hand_poses.left_pose.pos_xyz =  left_pos
        # eef_pose_msg.hand_poses.left_pose.quat_xyzw = [0.0,-0.70682518,0.0,0.70738827] # 四元数
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_angle
        # print("left_elbow: ", left_elbow)
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow # 设置成 0.0 时,不会被使用

        # 设置右手末端执行器的位置和姿态
        # eef_pose_msg.hand_poses.right_pose.pos_xyz =  np.array([0.3,-0.18,0.11988012])
        eef_pose_msg.hand_poses.right_pose.pos_xyz =  right_pos
        # eef_pose_msg.hand_poses.right_pose.quat_xyzw = [0.0,-0.70682518,0.0,0.70738827] # 四元数
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_angle
        # eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)  # 设置成 0.0 时,不会被使用
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow  # 设置成 0.0 时,不会被使用

        # 调用 IK 服务
        res = self._call_ik_srv(eef_pose_msg)
        if(res.success):
            return 57.325 * np.array(res.hand_poses.left_pose.joint_angles + res.hand_poses.right_pose.joint_angles, dtype=np.float64) 
        else:
            return False

    def _call_ik_srv(self, eef_pose_msg):
        rospy.wait_for_service('/ik/two_arm_hand_pose_cmd_srv')
        try:
            ik_srv = rospy.ServiceProxy('/ik/two_arm_hand_pose_cmd_srv', twoArmHandPoseCmdSrv)
            res = ik_srv(eef_pose_msg)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False, []        


    # State feedback
    def get_current_positions(self):
        '''
        Return:
            current_arm_joint_pos: numpy array of shape (14,) for left and right arm joints in degrees
            current_dexhand_joint_pos: numpy array of shape (12,) for left and right dex-hand joints in degrees
            current_head_joint_pos: numpy array of shape (2,) for head yaw and pitch in degrees
        '''
        return self._current_arm_joint_pos, self._current_dexhand_joint_pos, self._current_head_joint_pos


    def get_current_images(self):
        '''
        Return:
            head_color_img: numpy array for head color image
            head_depth_img: numpy array for head depth image
            left_hand_wrist_color_img: numpy array for left hand wrist color image
            left_hand_wrist_depth_img: numpy array for left hand wrist depth image
            right_hand_wrist_color_img: numpy array for right hand wrist color image
            right_hand_wrist_depth_img: numpy array for right hand wrist depth image
        '''
        return self.head_color_img, self.head_depth_img, self.left_hand_wrist_color_img, self.left_hand_wrist_depth_img, self.right_hand_wrist_color_img, self.right_hand_wrist_depth_img

    def get_current_tactile_state(self):
        '''
        Return:
            tactile_state: dict for tactile state
                {
                    "left_hand_tactile_state": numpy array of shape (5, 12) for left hand tactile data,
                    "right_hand_tactile_state": numpy array of shape (5, 12) for right hand tactile data,
                    "left_hand_contact": int for left hand contact state (0 or 1),
                    "right_hand_contact": int for right hand contact state (0 or 1),
                }
        '''
        return self.tactile_state


    def sleep(self):
        self._rate.sleep()
    
    # def _if_at_a_pos(self, pos_1, pos_2):
    #     if len(pos_1) != len(pos_2):
    #         raise ValueError("两个数组的长度必须相等")
    
    #     # 遍历数组，检查每对元素的差值
    #     for a, b in zip(pos_1, pos_2):
    #         if abs(a - b) >= 2:
    #             return False  # 一旦发现差值大于等于2，返回False
        
    #     return True


    def _head_color_image_callback(self, data):
        img = self._imgmsg_to_numpy(data)
        if img is None:
            return
        self.head_color_img = img[::-1, ::-1]

    def _head_depth_image_callback(self, data):
        img = self._imgmsg_to_numpy(data)
        if img is None:
            return
        self.head_depth_img = img[::-1, ::-1]

    def _left_hand_wrist_color_image_callback(self, data):
        self.left_hand_wrist_color_img = self._imgmsg_to_numpy(data)

    def _left_hand_wrist_depth_image_callback(self, data):
        self.left_hand_wrist_depth_img = self._imgmsg_to_numpy(data)

    def _right_hand_wrist_color_image_callback(self, data):
        self.right_hand_wrist_color_img = self._imgmsg_to_numpy(data)

    def _right_hand_wrist_depth_image_callback(self, data):
        self.right_hand_wrist_depth_img = self._imgmsg_to_numpy(data)

    def _imgmsg_to_numpy(self, data: Image):

        encoding = data.encoding.lower()
        if encoding in ("rgb8", "bgr8"):
            dtype, channels = np.uint8, 3
        elif encoding in ("rgba8", "bgra8"):
            dtype, channels = np.uint8, 4
        elif encoding in ("mono8", "8uc1"):
            dtype, channels = np.uint8, 1
        elif encoding in ("mono16", "16uc1"):
            dtype, channels = np.uint16, 1
        elif encoding in ("16sc1",):
            dtype, channels = np.int16, 1
        elif encoding in ("32fc1",):
            dtype, channels = np.float32, 1
        elif encoding in ("32fc3",):
            dtype, channels = np.float32, 3
        else:
            rospy.logwarn(f"Unsupported image encoding: {data.encoding}")
            return None

        height = int(data.height)
        width = int(data.width)
        bytes_per_pixel = np.dtype(dtype).itemsize * channels
        row_bytes = width * bytes_per_pixel
        step = int(data.step)
        buf = np.frombuffer(data.data, dtype=np.uint8)

        if step == row_bytes:
            img = buf.view(dtype).reshape(height, width, channels) if channels > 1 else buf.view(dtype).reshape(height, width)
            return img

        # Handle row padding
        if step < row_bytes:
            rospy.logwarn(f"Invalid image step: step={step} < row_bytes={row_bytes}")
            return None

        row_view = buf.reshape(height, step)
        trimmed = row_view[:, :row_bytes].reshape(height, row_bytes)
        img = trimmed.view(dtype)
        if channels > 1:
            return img.reshape(height, width, channels)
        return img.reshape(height, width)

    def _head_joint_pos_callback(self, msg):
        # robotHeadMotionData: joint_data[0]=yaw, joint_data[1]=pitch (degrees)
        self._current_head_joint_pos = np.array(msg.joint_data).astype(np.float64)
    
    def _hand_joint_pos_callback(self, msg):
        '''
        :position: first 6 values belong to right hand, last 6 values belong to left hand.
        '''
        # Dexhand joint positions from /dexhand/state (units/order per driver and msg.name).
        self._current_dexhand_joint_pos = np.array(msg.position).astype(np.float64)
        # TODO: add joint velocity and strom effort monitoring
        # self._current_dexhand_joint_vel = np.array(msg.velocity).astype(np.float64)
        # self._current_dexhand_joint_effort = np.array(msg.effort).astype(np.float64)

    def _hand_tactile_state_callback(self, msg):

        self.left_hand_tactile_state = self._parse_touch_hand_state(msg.left_hand)
        self.right_hand_tactile_state = self._parse_touch_hand_state(msg.right_hand)

        # thumb + index (first two fingers), sum all 3 normal-force channels, threshold at 1N.
        left_thumb_index_normal = np.sum(self.left_hand_tactile_state[0:2, 0:3])
        right_thumb_index_normal = np.sum(self.right_hand_tactile_state[0:2, 0:3])

        self.left_hand_contact = int(left_thumb_index_normal >= self._contact_threshold_n)
        self.right_hand_contact = int(right_thumb_index_normal >= self._contact_threshold_n)

        self.tactile_state = {
            "left_hand_tactile_state": self.left_hand_tactile_state,
            "right_hand_tactile_state": self.right_hand_tactile_state,
            "left_hand_contact": self.left_hand_contact,
            "right_hand_contact": self.right_hand_contact,
        }

        # if self.left_hand_contact or self.right_hand_contact:
        #     cprint(f"Contact detected! Left hand state: {self.left_hand_tactile_state}, Right hand state: {self.right_hand_tactile_state}", "green")  

    def _parse_touch_hand_state(self, hand_data):
        """
        Returns a (5, 12) tactile matrix:
            rows 0-4: thumb -> pinky
            cols 0-2: normal_force1/2/3 (N)
            cols 3-5: tangential_force1/2/3 (N)
            cols 6-8: tangential_direction1/2/3 (deg)
            cols 9-11: self_proximity1/2, mutual_proximity
        """
        tactile = np.zeros((5, 12), dtype=np.float64)

        if isinstance(hand_data, (list, tuple)):
            finger_states = list(hand_data)
        else:
            finger_states = [hand_data]

        for finger_idx, finger in enumerate(finger_states[:5]):
            tactile[finger_idx, 0] = getattr(finger, "normal_force1", 0.0) / 100.0
            tactile[finger_idx, 1] = getattr(finger, "normal_force2", 0.0) / 100.0
            tactile[finger_idx, 2] = getattr(finger, "normal_force3", 0.0) / 100.0
            tactile[finger_idx, 3] = getattr(finger, "tangential_force1", 0.0) / 100.0
            tactile[finger_idx, 4] = getattr(finger, "tangential_force2", 0.0) / 100.0
            tactile[finger_idx, 5] = getattr(finger, "tangential_force3", 0.0) / 100.0
            tactile[finger_idx, 6] = float(getattr(finger, "tangential_direction1", -1.0))
            tactile[finger_idx, 7] = float(getattr(finger, "tangential_direction2", -1.0))
            tactile[finger_idx, 8] = float(getattr(finger, "tangential_direction3", -1.0))
            tactile[finger_idx, 9] = float(getattr(finger, "self_proximity1", 0.0))
            tactile[finger_idx, 10] = float(getattr(finger, "self_proximity2", 0.0))
            tactile[finger_idx, 11] = float(getattr(finger, "mutual_proximity", 0.0))

        return tactile
    
    def _arm_joint_pos_callback(self, msg):
        # Last 14 entries are arm joints in radians; convert to degrees for internal use.
        self._current_arm_joint_pos = np.degrees(np.asarray(msg.data[-14:], dtype=np.float64))
        # print("Current arm joint positions (degrees): ", self._current_arm_joint_pos)

    # def _thread_job(self):
    #     rospy.spin()


    

    def hand_pass_left2right(self):
        self.prepared_flag = False
        # self.ges_arm_prepare = np.array([-10, 20, -30, -100, -30, 0, 0,    -10, -20, 30, -100, 30, 0, 0], dtype=np.float64)
        # self.set_both_arm_traj([np.array([-20, 10, -40, -90, -30, 0, 0,    -10, -20, 30, -100, 30, 0, 0], dtype=np.float64)]) #single arm
        self.set_both_arm_traj([self.ges_arm_left_twist_away])
        self.set_both_hands_action(self.ges_hand_release + self.ges_hand_pinch)
        self.set_both_arm_traj([np.array([-20, 10, -40, -95, -30, 0, 0,    -20, -10, 40, -90, 30, 0, 0], dtype=np.float64)]) #both arms
        self.set_both_hands_action(self.ges_hand_pinch + self.ges_hand_pinch)
        self.set_both_hands_action(self.ges_hand_pinch + self.ges_hand_release)
        self.set_both_arm_traj([self.ges_arm_right_twist_away])
        self.set_both_hands_action(self.ges_hand_release + self.ges_hand_release)
        self.set_both_arm_traj([self.ges_arm_prepare])
        return

    def right_hand_grab(self):
        self.prepared_flag = False
        self.set_both_hands_action(self.ges_hand_grab + self.ges_hand_release)
        self.set_both_arm_traj([self.ges_arm_right_twist_away])
        self.set_both_hands_action(self.ges_hand_release + self.ges_hand_release)
        self.set_both_arm_traj([self.ges_arm_prepare])
        self.prepared_flag = True
        
    def left_hand_grab(self):
        self.prepared_flag = False
        self.set_both_hands_action(self.ges_hand_release + self.ges_hand_grab)
        self.set_both_arm_traj([self.ges_arm_left_twist_away])
        self.set_both_hands_action(self.ges_hand_release + self.ges_hand_release)
        self.set_both_arm_traj([self.ges_arm_prepare])
        self.prepared_flag = True

    def demo_grab(self):
        self.prepare()
        rospy.sleep(10)
        self.left_hand_grab()
        self.right_hand_grab()
        return
    
    def demo_trans_between_hands(self):
        self.prepare()
        self.hand_pass_left2right()
        return


def demo():

    controller = Controller("test.config")
    # controller.set_both_arm_traj([controller.ges_arm_prepare])
    controller.set_both_arm_traj(np.array([0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0], dtype=np.float64))
    # rospy.sleep(0.1)
    # controller.prepare()
    # rospy.loginfo('go to ready pos')
    # rospy.sleep(0.5)
    # controller.demo_grab()
    # controller.set_both_arm_traj([np.array([-20, 10, -40, -90, -30, 0, 0,    -20, -10, 40, -90, 30, 0, 0], dtype=np.float64)]) #both arms
    # controller.demo_trans_between_hands()


    # TODO: a demo that enable a hard-code grasping
    # result = controller.arm_pos2angle(np.array([0.45,0.25,0.12]), [0.0,-0.707,0.0,0.707], np.array([0.45,-0.25,0.12]), [0.0,-0.707,0.0,0.707])
    # controller.set_both_arm_traj([result])
    # result = controller.arm_pos2angle(np.array([0,0.2,-0.2]), [0.0707,0.0,0.707,0.0], np.array([0.5,-0.25,0.15]), [0.0,-0.707,0.0,0.707])
    # controller.set_both_arm_traj([result])
    # result = controller.arm_pos2angle(np.array([0.5,0.16,0.15]), [0.0,-0.707,0.0,0.707], np.array([0.5,-0.16,0.15]), [0.0,-0.707,0.0,0.707], np.array([0,0.3,0.15]), np.array([0,-0.3,0.15]))
    # controller.set_both_arm_traj([result])
    # sleep(6)
    # controller.set_both_arm_traj([np.array([0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0], dtype=np.float64)])


    # controller.reset()
    rospy.loginfo('reset to init pos')

    # rospy.sleep(2)
    # rospy.loginfo('set head target init [0, 0]')
    # controller.set_head_target_pos([0, 0])

    # rospy.sleep(2)
    # rospy.loginfo('set head target down [0, 20]')
    # controller.set_head_target_pos([0, 20])

    # controller.set_both_arm_traj([57* np.array([0.4558641051538501, -0.32665322953864107, -0.019143323431253682, -1.9107488189208253, -0.28226964333490445, -0.11980679750364086, -0.10016324461696151, 0.35267708300027645, -0.13019770255671365, 0.4542355485690402, -1.9027938731908716, -0.014763047786313856, -0.450232455512944, -0.04712842685697043], dtype=np.float64)])
    
    # controller.prepare()


def camera_test():
    
    controller = Controller(config={'test':0})
    rospy.loginfo('from prepare go to init pose')
    controller.reset()
    controller.sleep()
    
    rospy.loginfo('go to prepare pose')
    controller.prepare()
    controller.sleep()

    print("head color", controller.head_color_img.shape)
    print("left color", controller.left_hand_wrist_color_img.shape)
    print("right color", controller.right_hand_wrist_color_img.shape)

    print("head depth", controller.head_depth_img.shape)
    print("left depth", controller.left_hand_wrist_depth_img.shape)
    print("right depth", controller.left_hand_wrist_depth_img.shape)

    # rospy.loginfo('reset to initial pose')
    # controller.reset()

    output_dir = os.path.dirname(os.path.abspath(__file__))

    def _save_png(filename, img):
        if img is None:
            return
        arr = img
        if arr.dtype.kind == "f":
            max_val = float(np.nanmax(arr)) if arr.size else 0.0
            if max_val > 0:
                arr = np.clip(arr / max_val * 65535.0, 0, 65535).astype(np.uint16)
            else:
                arr = np.zeros(arr.shape, dtype=np.uint16)
        elif arr.dtype.kind == "i":
            arr = arr.astype(np.uint16, copy=False)
        cv2.imwrite(os.path.join(output_dir, filename), arr)

    _save_png("head_color.png", controller.head_color_img)
    _save_png("head_depth.png", controller.head_depth_img)
    _save_png("left_wrist_color.png", controller.left_hand_wrist_color_img)
    _save_png("left_wrist_depth.png", controller.left_hand_wrist_depth_img)
    _save_png("right_wrist_color.png", controller.right_hand_wrist_color_img)
    _save_png("right_wrist_depth.png", controller.right_hand_wrist_depth_img)


def prepare_pose_test():
    
    controller = Controller(config={'test':0})
    
    # print(controller._current_head_joint_pos)
    # print(controller._current_arm_joint_pos)
    # print(controller._current_dexhand_joint_pos)

    print("---------------------------------")
    rospy.loginfo('go to init pose')
    controller.reset(initermediate=False)
    controller.sleep()

    # print(controller._current_head_joint_pos)
    # print(controller._current_arm_joint_pos)
    # print(controller._current_dexhand_joint_pos)

    rospy.sleep(5)


    print("---------------------------------")
    rospy.loginfo('go to prepare pose')
    controller.prepare()
    controller.sleep()

    # print(controller._current_head_joint_pos)
    # print(controller._current_arm_joint_pos)
    # print(controller._current_dexhand_joint_pos)

    rospy.sleep(5)

    print("---------------------------------")
    rospy.loginfo('go to init pose')
    controller.reset()
    controller.sleep()

    # print(controller._current_head_joint_pos)
    # print(controller._current_arm_joint_pos)
    # print(controller._current_dexhand_joint_pos)


def dex_hand_tactile_test():

    controller = Controller(config={'test':0})

    print("---------------------------------")
    rospy.loginfo('go to init pose')
    controller.reset()
    controller.sleep()

    # print(controller._current_head_joint_pos)
    # print(controller._current_arm_joint_pos)
    # print(controller._current_dexhand_joint_pos)

    rospy.sleep(5)


    print("---------------------------------")
    rospy.loginfo('go to prepare pose')
    controller.prepare()
    controller.sleep()

    tactile_state = controller.get_current_tactile_state()
    print("tactile_state:", tactile_state)

    # Record tactile force for a window, then draw 10 subplots (5 left + 5 right fingers).
    record_duration_sec = 20.0
    sample_hz = 20.0
    sample_rate = rospy.Rate(sample_hz)
    finger_names = ["thumb", "index", "middle", "ring", "pinky"]

    timestamps = []
    left_force_mag_hist = []
    right_force_mag_hist = []

    rospy.loginfo(f"Start tactile recording for {record_duration_sec:.1f}s at {sample_hz:.1f}Hz")
    start_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        elapsed = now - start_time
        if elapsed > record_duration_sec:
            break

        tactile_state = controller.get_current_tactile_state()
        if tactile_state is None:
            sample_rate.sleep()
            continue

        left_tactile = np.array(tactile_state["left_hand_tactile_state"], dtype=np.float64, copy=True)
        right_tactile = np.array(tactile_state["right_hand_tactile_state"], dtype=np.float64, copy=True)

        left_force_mag = np.linalg.norm(left_tactile[:, 0:3], axis=1)
        right_force_mag = np.linalg.norm(right_tactile[:, 0:3], axis=1)

        timestamps.append(elapsed)
        left_force_mag_hist.append(left_force_mag)
        right_force_mag_hist.append(right_force_mag)
        sample_rate.sleep()

    if len(timestamps) == 0:
        rospy.logwarn("No tactile samples recorded. Skip plotting.")
        return

    left_force_mag_hist = np.asarray(left_force_mag_hist, dtype=np.float64)
    right_force_mag_hist = np.asarray(right_force_mag_hist, dtype=np.float64)
    timestamps = np.asarray(timestamps, dtype=np.float64)

    fig, axes = plt.subplots(10, 1, figsize=(14, 24), sharex=True)
    for i in range(5):
        axes[i].plot(timestamps, left_force_mag_hist[:, i], color="tab:blue", linewidth=1.8)
        axes[i].set_ylabel(f"L-{finger_names[i]} (N)")
        axes[i].grid(True, linestyle="--", alpha=0.35)

    for i in range(5):
        ax_idx = i + 5
        axes[ax_idx].plot(timestamps, right_force_mag_hist[:, i], color="tab:orange", linewidth=1.8)
        axes[ax_idx].set_ylabel(f"R-{finger_names[i]} (N)")
        axes[ax_idx].grid(True, linestyle="--", alpha=0.35)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Dex Hand Tactile Normal-Force Magnitude per Finger", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.985])

    out_dir = os.path.dirname(os.path.abspath(__file__))
    ts_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    fig_path = os.path.join(out_dir, f"tactile_force_timeseries_{ts_tag}.png")
    csv_path = os.path.join(out_dir, f"tactile_force_timeseries_{ts_tag}.csv")

    fig.savefig(fig_path, dpi=300)
    plt.close(fig)

    data_to_save = np.column_stack([timestamps, left_force_mag_hist, right_force_mag_hist])
    header = "time_s," + ",".join([f"left_{name}_norm_N" for name in finger_names] + [f"right_{name}_norm_N" for name in finger_names])
    np.savetxt(csv_path, data_to_save, delimiter=",", header=header, comments="")

    rospy.loginfo(f"Tactile figure saved to: {fig_path}")
    rospy.loginfo(f"Tactile data saved to: {csv_path}")


def demo_hand_tactile_visualization(controller=None, stop_event=None):
    if controller is None:
        controller = Controller(config={'test': 0})
    if stop_event is None:
        stop_event = threading.Event()

    print("---------------------------------")
    rospy.loginfo('go to init pose')
    controller.reset()
    controller.sleep()

    # print(controller._current_head_joint_pos)
    # print(controller._current_arm_joint_pos)
    # print(controller._current_dexhand_joint_pos)
    rospy.sleep(5)


    print("---------------------------------")
    rospy.loginfo('go to prepare pose')
    controller.prepare()
    controller.sleep()

    finger_names = ["thumb", "index", "middle", "ring", "pinky"]
    proximity_names = ["self_proximity1", "self_proximity2", "mutual_proximity"]
    sample_hz = 10.0
    sample_rate = rospy.Rate(sample_hz)
    timestamps = []
    left_force_mag_hist = []
    right_force_mag_hist = []
    left_proximity_hist = []
    right_proximity_hist = []
    left_raw_hist = []
    right_raw_hist = []

    def _run_actions():

        rospy.sleep(5.0)
        # Close right hand for 5s.
        controller.set_both_hands_action(controller.ges_hand_grab + controller.ges_hand_release)
        rospy.sleep(5.0)

        # Release hands, wait 5s.
        controller.set_both_hands_action(controller.ges_hand_release + controller.ges_hand_release)
        rospy.sleep(5.0)

        # Close left hand for 5s.
        controller.set_both_hands_action(controller.ges_hand_release + controller.ges_hand_grab)
        rospy.sleep(5.0)

        # Release both hand and wait 1s.
        controller.set_both_hands_action(controller.ges_hand_release + controller.ges_hand_release)
        rospy.sleep(1.0)

        stop_event.set()

    rospy.loginfo("Start tactile recording during hand sequence")
    stop_event.clear()
    action_thread = threading.Thread(target=_run_actions, daemon=True)
    action_thread.start()
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown() and not stop_event.is_set():
        tactile_state = controller.get_current_tactile_state()
        if tactile_state is None:
            sample_rate.sleep()
            continue

        elapsed = rospy.Time.now().to_sec() - start_time
        left_tactile = np.array(tactile_state["left_hand_tactile_state"], dtype=np.float64, copy=True)
        right_tactile = np.array(tactile_state["right_hand_tactile_state"], dtype=np.float64, copy=True)
        left_force_mag = np.linalg.norm(left_tactile[:, 0:3], axis=1)
        right_force_mag = np.linalg.norm(right_tactile[:, 0:3], axis=1)
        left_proximity = left_tactile[:, 9:12]
        right_proximity = right_tactile[:, 9:12]

        timestamps.append(elapsed)
        left_force_mag_hist.append(left_force_mag)
        right_force_mag_hist.append(right_force_mag)
        left_proximity_hist.append(left_proximity)
        right_proximity_hist.append(right_proximity)
        left_raw_hist.append(left_tactile)
        right_raw_hist.append(right_tactile)
        sample_rate.sleep()

    action_thread.join(timeout=2.0)

    if len(timestamps) == 0:
        rospy.logwarn("No tactile samples recorded during hand sequence.")
    else:
        timestamps = np.asarray(timestamps, dtype=np.float64)
        left_force_mag_hist = np.asarray(left_force_mag_hist, dtype=np.float64)
        right_force_mag_hist = np.asarray(right_force_mag_hist, dtype=np.float64)
        left_proximity_hist = np.asarray(left_proximity_hist, dtype=np.float64)
        right_proximity_hist = np.asarray(right_proximity_hist, dtype=np.float64)
        left_raw_hist = np.asarray(left_raw_hist, dtype=np.float64)
        right_raw_hist = np.asarray(right_raw_hist, dtype=np.float64)

        fig_force, axes_force = plt.subplots(5, 1, figsize=(14, 18), sharex=True)
        for i in range(5):
            axes_force[i].plot(timestamps, left_force_mag_hist[:, i], color="tab:blue", linewidth=1.8, label="left")
            axes_force[i].plot(timestamps, right_force_mag_hist[:, i], color="tab:orange", linewidth=1.8, label="right")
            axes_force[i].set_ylabel(f"{finger_names[i]} (N)")
            axes_force[i].grid(True, linestyle="--", alpha=0.35)
            axes_force[i].legend(loc="upper right")
        axes_force[-1].set_xlabel("Time (s)")
        fig_force.suptitle("Dex Hand Tactile Force Magnitude During Hand Sequence", fontsize=14)
        fig_force.tight_layout(rect=[0, 0, 1, 0.985])

        fig_prox, axes_prox = plt.subplots(5, 1, figsize=(14, 18), sharex=True)
        for i in range(5):
            for j in range(3):
                axes_prox[i].plot(
                    timestamps,
                    left_proximity_hist[:, i, j],
                    linewidth=1.6,
                    label=f"left_{proximity_names[j]}",
                )
                axes_prox[i].plot(
                    timestamps,
                    right_proximity_hist[:, i, j],
                    linewidth=1.6,
                    linestyle="--",
                    label=f"right_{proximity_names[j]}",
                )
            axes_prox[i].set_ylabel(f"{finger_names[i]}")
            axes_prox[i].grid(True, linestyle="--", alpha=0.35)
            axes_prox[i].legend(loc="upper right", ncol=2, fontsize=8)
        axes_prox[-1].set_xlabel("Time (s)")
        fig_prox.suptitle("Dex Hand Proximity Signals During Hand Sequence", fontsize=14)
        fig_prox.tight_layout(rect=[0, 0, 1, 0.985])

        out_dir = os.path.dirname(os.path.abspath(__file__))
        ts_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
        force_fig_path = os.path.join(out_dir, f"demo_hand_sequence_force_{ts_tag}.png")
        prox_fig_path = os.path.join(out_dir, f"demo_hand_sequence_proximity_{ts_tag}.png")
        csv_path = os.path.join(out_dir, f"demo_hand_sequence_tactile_raw_{ts_tag}.csv")

        fig_force.savefig(force_fig_path, dpi=300)
        fig_prox.savefig(prox_fig_path, dpi=300)
        plt.close(fig_force)
        plt.close(fig_prox)

        data_cols = [timestamps]
        header_cols = ["time_s"]
        channel_names = [
            "normal_force1_N",
            "normal_force2_N",
            "normal_force3_N",
            "tangential_force1_N",
            "tangential_force2_N",
            "tangential_force3_N",
            "tangential_direction1_deg",
            "tangential_direction2_deg",
            "tangential_direction3_deg",
            "self_proximity1",
            "self_proximity2",
            "mutual_proximity",
        ]
        for hand_name, hand_hist in [("left", left_raw_hist), ("right", right_raw_hist)]:
            for finger_idx, finger_name in enumerate(finger_names):
                for channel_idx, channel_name in enumerate(channel_names):
                    data_cols.append(hand_hist[:, finger_idx, channel_idx])
                    header_cols.append(f"{hand_name}_{finger_name}_{channel_name}")

        data_to_save = np.column_stack(data_cols)
        header = ",".join(header_cols)
        np.savetxt(csv_path, data_to_save, delimiter=",", header=header, comments="")

        rospy.loginfo(f"Demo force figure saved to: {force_fig_path}")
        rospy.loginfo(f"Demo proximity figure saved to: {prox_fig_path}")
        rospy.loginfo(f"Demo tactile raw data saved to: {csv_path}")

    print("---------------------------------")
    rospy.loginfo('go to init pose')
    controller.reset()
    controller.sleep()


def demo_dexhand_action():

    controller = Controller(config={'test':0})

    print("---------------------------------")
    rospy.loginfo('go to init pose')
    controller.reset()
    controller.sleep()
    print("---------------------------------")
    rospy.loginfo('go to prepare pose')
    controller.prepare()
    controller.sleep()

    rospy.sleep(3)
    print("---------------------------------")
    rospy.loginfo('left hand grasp')
    controller.set_both_hands_action(controller.ges_hand_grab + controller.ges_hand_release)
    print(f'left hand pos {controller._current_dexhand_joint_pos[:6]}, right hand pos {controller._current_dexhand_joint_pos[6:]}')

    rospy.sleep(3)
    print("---------------------------------")
    rospy.loginfo('right hand grasp')
    controller.set_both_hands_action(controller.ges_hand_release + controller.ges_hand_grab)
    print(f'left hand pos {controller._current_dexhand_joint_pos[:6]}, right hand pos {controller._current_dexhand_joint_pos[6:]}')

    rospy.sleep(3)

    print("---------------------------------")
    rospy.loginfo('go to init pose')
    controller.reset()
    controller.sleep()
    


if __name__ == "__main__":

    cprint('Run Kuavo demo test.', 'cyan')
    # sleep(1)
    # prepare_pose_test()

    # dex_hand_tactile_test()

    # demo_hand_tactile_visualization()

    demo_dexhand_action()

    cprint('demo test finished.', 'green')
