#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Upper body base controller class
import numpy as np
import rospy
from typing import Dict, Union, List, Tuple
from time import sleep
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Image
from kuavo_msgs.msg import robotHeadMotionData, dexhandCommand, twoArmHandPoseCmd, ikSolveParam
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse, twoArmHandPoseCmdSrv
import sys
import threading
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
        self._current_head_joint_pos = np.zeros(2)
        self._current_arm_joint_pos = np.zeros(14)
        self._current_hand_joint_pos = np.zeros(12)
        self._current_joint_pos = np.concatenate([self._current_head_joint_pos, 
                                                  self._current_arm_joint_pos, 
                                                  self._current_hand_joint_pos])
        self.head_color_img = None
        self.head_depth_img = None
        self.left_hand_wrist_color_img = None
        self.left_hand_wrist_depth_img = None
        self.right_hand_wrist_color_img = None
        self.right_hand_wrist_depth_img = None
        
        # init pose
        self.ges_hand_grab = [60, 60, 60, 60, 60, 60]
        self.ges_hand_release = [0, 0, 0, 0, 0, 0]
        self.ges_hand_pinch = [60, 60, 60, 0, 0, 0]
        self.ges_arm_prepare = np.array([-10, 20, -30, -100, -30, 0, 0,    -10, -20, 30, -100, 30, 0, 0], dtype=np.float64)
        self.ges_arm_left_twist_away = np.array([-10, 20, 50, -90, -30, 0, 0,    -10, -20, 30, -100, 30, 0, 0], dtype=np.float64)
        self.ges_arm_right_twist_away = np.array([-10, 20, -30, -100, -30, 0, 0,    -10, -20, -50, -90, 30, 0, 0], dtype=np.float64)
        
        self.init_pos = np.array([  0, 0, 0, 0, 0, 0, 0,                        # left
                                    0, 0, 0, 0, 0, 0, 0], dtype=np.float64)    # right
        self.intermediate_pos = np.array([  45, 20, 0, -90, 0, 0, 0,                  
                                            45, -20, 0, -90, 0, 0, 0], dtype=np.float64)
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
        # head
        self.head_pose_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
        # arms
        self.arm_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
        # dex-hands
        self.hand_pub = rospy.Publisher('/dexhand/command', dexhandCommand, queue_size=10)
        
        # TODO: sub joint states: head, arms, dex-hands, give to self._current_joint_pos
        self.joint_sub =  rospy.Subscriber('/humanoid_controller/optimizedState_mrt/joint_pos', Float64MultiArray, self._arm_joint_pos_callback)
        self.dexhand_sub = rospy.Subscriber('/dexhand/state', JointState, self._hand_joint_pos_callback)

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
        self._thread = threading.Thread(target=self._thread_job)
        self._thread.start()
        rospy.loginfo("Kuavo base controller initialized.")
        
        
    # ============== Reset & prepare ===============
    def reset(self):
        '''Reset to robot initial pose'''
        self.set_head_target_pos([0, 0]) # head down
        self.set_both_hands_action(self.ges_hand_release + self.ges_hand_release)
        '''Reset to init zero pose'''
        action_list = [self.prepare_pos, self.intermediate_pos, self.init_pos]
        self.set_both_arm_traj(action_list)
        rospy.loginfo('reset to init pose')
        self._rate.sleep()
        return True

    def prepare(self):
        '''from init pose go to prepare pose'''
        self.set_head_target_pos([0, 20]) # head down
        self.set_both_hands_action(self.ges_hand_release + self.ges_hand_release)
        '''go to prepare pose for table-top manipulation'''
        action_list = [self.init_pos, self.intermediate_pos, self.prepare_pos]
        self.set_both_arm_traj(action_list)
        rospy.loginfo('prepare for manipulation')
        self._rate.sleep()
        return True
    
    def go_to_prepare_pose(self):
        '''from random pose go to prepare pose'''
        self.set_both_hands_action(self.ges_hand_release + self.ges_hand_release)
        '''go to prepare pose for table-top manipulation'''
        action_list = [self.prepare_pos]
        self.set_both_arm_traj(action_list)
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

    # Arms
    def set_both_arm_traj(self, action_list:List):

        current_arm_joint_pos_data = self.get_current_positions()[0]
        action_list.insert(0, current_arm_joint_pos_data)
        # print(position_list)
        # position_list = copy.deepcopy(position_list)
        step = len(action_list)
        for i in range(step - 1):
            print(current_arm_joint_pos_data)
            # print(i, ":", position_list[i])
            # print(i + 1, ":", position_list[i + 1])
            diff = np.sum(np.abs(action_list[i + 1] - action_list[i]))
            print(diff)
            duration = diff / 60
            if duration == 0:
                continue
            # my_arm_move(position_list[i], position_list[i + 1], duration)
            self.set_arm_joint_move(action_list[i], action_list[i + 1], 2)
    
        return True

    def set_arm_joint_move(self, current_position, target_position, duration):
        """
        控制机械臂平滑地从当前关节角度运动到目标位置。
        
        :param target_position: 目标关节角度的numpy数组 (长度应为14)
        :param duration: 运动持续时间（秒）
        """
        msg = JointState()
        msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]  # 关节名称列表

        # 设置时间参数
        steps = int(duration * self.hz)  # 计算步骤数

        # 计算每一步的角度变化量
        delta_position = (target_position - current_position) / steps

        # 运动过程中逐步更新关节位置
        for step in range(steps):
            # 逐步更新每个关节的角度
            current_position += delta_position
            msg.header.stamp = rospy.Time.now()  # 当前时间戳
            msg.position = current_position.tolist()  # 更新关节位置

            self.arm_pub.publish(msg)  # 发布关节状态
            self._rate.sleep()

        # 最后一次发布，确保最终目标位置被发布
        msg.position = target_position.tolist()
        self.arm_pub.publish(msg)
    
    
    # Dex Hands
    def set_both_hands_action(self, hand_joints):
        '''
        Order: 
            thumb, thumb_aux, index, middle, ring, pinky
            right = hand_joints[:6]
            left = hand_joints[-6:]
            both_hand_msg.data = right + left # 一共 12 个元素'''
        
        while not rospy.is_shutdown():

            both_hand_msg = dexhandCommand()
            both_hand_msg.control_mode = 0  # POSITION_CONTROL
            both_hand_msg.data = hand_joints
            self.hand_pub.publish(both_hand_msg)

            if np.mean(np.array(self.get_current_positions()[1]) - np.array(hand_joints)) < 2:
                sleep(1)
                break
            
            self._rate.sleep()


    # State retrival
    def get_current_positions(self):
        '''
        Return:
            1: 
            2: 
            3: ...
        '''
        return self._current_arm_joint_pos, self._current_hand_joint_pos



    def arm_pos2angle(self, left_pos, left_angle, right_pos, right_angle, left_elbow = np.zeros(3), right_elbow = np.zeros(3)):
        def call_ik_srv(eef_pose_msg):
            rospy.wait_for_service('/ik/two_arm_hand_pose_cmd_srv')
            try:
                ik_srv = rospy.ServiceProxy('/ik/two_arm_hand_pose_cmd_srv', twoArmHandPoseCmdSrv)
                res = ik_srv(eef_pose_msg)
                return res
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False, []
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
        print("left_elbow: ", left_elbow)
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow # 设置成 0.0 时,不会被使用

        # 设置右手末端执行器的位置和姿态
        # eef_pose_msg.hand_poses.right_pose.pos_xyz =  np.array([0.3,-0.18,0.11988012])
        eef_pose_msg.hand_poses.right_pose.pos_xyz =  right_pos
        # eef_pose_msg.hand_poses.right_pose.quat_xyzw = [0.0,-0.70682518,0.0,0.70738827] # 四元数
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_angle
        # eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)  # 设置成 0.0 时,不会被使用
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow  # 设置成 0.0 时,不会被使用

        # 调用 IK 服务
        res = call_ik_srv(eef_pose_msg)
        if(res.success):
            return 57.325 * np.array(res.hand_poses.left_pose.joint_angles + res.hand_poses.right_pose.joint_angles, dtype=np.float64) 
        else:
            return False
        
    
    def _if_at_a_pos(self, pos_1, pos_2):
        if len(pos_1) != len(pos_2):
            raise ValueError("两个数组的长度必须相等")
    
        # 遍历数组，检查每对元素的差值
        for a, b in zip(pos_1, pos_2):
            if abs(a - b) >= 2:
                return False  # 一旦发现差值大于等于2，返回False
        
        return True

    def _head_color_image_callback(self, data):
        self.head_color_img = data
        # TODO: postprocess image data

    def _head_depth_image_callback(self, data):
        self.head_depth_img = data
        # TODO: postprocess image data

    def _left_hand_wrist_color_image_callback(self, data):
        self.left_hand_wrist_color_img = data
        # TODO: postprocess image data

    def _left_hand_wrist_depth_image_callback(self, data):
        self.left_hand_wrist_depth_img = data
        # TODO: postprocess image data

    def _right_hand_wrist_color_image_callback(self, data):
        self.right_hand_wrist_color_img = data
        # TODO: postprocess image data

    def _right_hand_wrist_depth_image_callback(self, data):
        self.right_hand_wrist_depth_img = data
        # TODO: postprocess image data

    def _head_joint_pos_callback(self, msg):
        self._current_head_joint_pos = np.array(msg.position).astype(np.float64)
    
    def _hand_joint_pos_callback(self, msg):
       self._current_hand_joint_pos = np.array(msg.position).astype(np.float64)
    
    def _arm_joint_pos_callback(self, msg):
        self._current_arm_joint_pos = (np.array(msg.data[-14:]) * 57.3).round().astype(np.float64)
        if self._if_at_a_pos(self._current_arm_joint_pos, self.ges_arm_prepare):
            self.prepared_flag = True
    
    def _thread_job(self):
        rospy.spin()


    

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
        sleep(10)
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
    controller.set_both_arm_traj([np.array([0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0], dtype=np.float64)])
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

def demo2():
    controller = Controller("test.config")
    # controller.set_both_arm_traj([np.array([0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0], dtype=np.float64)])
    # controller.set_head_target_pos(np.array([0.0, 0.0]))
    # # controller.reset()
    rospy.loginfo('go to prepare pose')
    controller.prepare()
    # rospy.loginfo('reset to initial pose')
    # controller.reset()

if __name__ == "__main__":

    cprint('Run Kuavo demo test.', 'cyan')
    # sleep(1)
    # demo()
    demo2()
    cprint('demo test finished.', 'green')
