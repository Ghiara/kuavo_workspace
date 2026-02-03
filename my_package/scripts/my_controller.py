# Upper body base controller class
import numpy as np
import rospy
from typing import Dict, Union, List, Tuple
from time import sleep
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import dexhandCommand
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
import sys
sys.path.insert(0, '/home/lab/kuavo-ros-opensource/src/my_package/scripts')
# from my_arm_move import my_arm_continuous_move, my_arm_to_origin
# from termcolor import cprint



class Controller():
    '''
    Kuavo base controller that enable uppper body control of kuavo 4pro maxB
    Left arm: 7-DoF
    Right arm: 7-DoF
    Head: 2-DoF
    Left dex hand: 6-DoF
    Right dex hand: 6-DoF
    '''

    def __init__(self, config, *args, **kwargs):
        self.config = config
        self.current_joint_positions = np.zeros(28+6+6) # include dex hands, TODO: joint definition
        self.gestures_config_init()
        self.flag_init()
        self.ros_init()
        
    def ros_init(self):
        rospy.init_node('dexhand_command_publisher')
        
        arm_traj_change_mode_client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)
        # 创建请求对象
        request = changeArmCtrlModeRequest()
        request.control_mode = 2  # 设置控制模式

        # 调用服务并获取响应
        response = arm_traj_change_mode_client(request)
        # 创建一个发布者，向 /dexhand/command 话题发送 dexhandCommand 类型消息
        self.hand_pub = rospy.Publisher('/dexhand/command', dexhandCommand, queue_size=10)
        self.arm_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
        def arm_joint_pos_callback(msg):
            # 提取最后 14 个数字
            global current_arm_joint_pos_data
            current_arm_joint_pos_data = (np.array(msg.data[-14:]) * 57.3).round().astype(np.float64)
            if self.if_at_a_pos(current_arm_joint_pos_data, self.ges_arm_prepare):
                self.prepared_flag = True
        rospy.Subscriber('/humanoid_controller/optimizedState_mrt/joint_pos', Float64MultiArray, arm_joint_pos_callback)
        def hand_joint_pos_callback(msg):
            # 提取最后 14 个数字
            global current_hand_joint_pos_data
            current_hand_joint_pos_data = np.array(msg.position).astype(np.float64)
        rospy.Subscriber('/dexhand/state', JointState, hand_joint_pos_callback)

        rate = rospy.Rate(10)
        rate.sleep()

    def gestures_config_init(self):
        self.ges_hand_grab = [60, 60, 60, 60, 60, 60]
        self.ges_hand_release = [0, 0, 0, 0, 0, 0]
        self.ges_hand_pinch = [60, 60, 60, 0, 0, 0]
        self.ges_arm_prepare = np.array([-10, 20, -30, -100, -30, 0, 0,    -10, -20, 30, -100, 30, 0, 0], dtype=np.float64)
        self.ges_arm_left_twist_away = np.array([-10, 20, 50, -90, -30, 0, 0,    -10, -20, 30, -100, 30, 0, 0], dtype=np.float64)
        self.ges_arm_right_twist_away = np.array([-10, 20, -30, -100, -30, 0, 0,    -10, -20, -50, -90, 30, 0, 0], dtype=np.float64)

    def flag_init(self):
        self.prepared_flag = False

    # Reset & prepare
    def reset(self):
        '''Reset to robot 0 initial position'''
        # my_arm_to_origin()
        raise NotImplementedError
    
    def if_at_a_pos(self, pos_1, pos_2):
        if len(pos_1) != len(pos_2):
            raise ValueError("两个数组的长度必须相等")
    
        # 遍历数组，检查每对元素的差值
        for a, b in zip(pos_1, pos_2):
            if abs(a - b) >= 2:
                return False  # 一旦发现差值大于等于2，返回False
        
        return True

    # Head 
    def set_head_action(self, action_delta):
        raise NotImplementedError

    # Arms
    def set_left_arm_action(self, action_delta):
        raise NotImplementedError
    
    def set_right_arm_action(self, action_delta):
        raise NotImplementedError

    def set_both_arm_traj(self, action_list):
        def my_arm_move(current_position, target_position, duration):
            """
            控制机械臂平滑地从当前关节角度运动到目标位置。
            
            :param target_position: 目标关节角度的numpy数组 (长度应为14)
            :param duration: 运动持续时间（秒）
            """
            # 当前关节位置
            # current_position = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
            
            # 创建关节状态消息
            msg = JointState()
            msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]  # 关节名称列表

            # 设置时间参数
            rate = 10  # 发布频率，单位 Hz
            steps = int(duration * rate)  # 计算步骤数

            # 计算每一步的角度变化量
            delta_position = (target_position - current_position) / steps

            # 运动过程中逐步更新关节位置
            for step in range(steps):
                # 逐步更新每个关节的角度
                current_position += delta_position
                msg.header.stamp = rospy.Time.now()  # 当前时间戳
                msg.position = current_position.tolist()  # 更新关节位置

                self.arm_pub.publish(msg)  # 发布关节状态
                rospy.sleep(1.0 / rate)  # 按照发布频率睡眠

            # 最后一次发布，确保最终目标位置被发布
            msg.position = target_position.tolist()
            self.arm_pub.publish(msg)

        def my_arm_continuous_move(position_list):
            current_arm_joint_pos_data = self.get_current_positions()[0]
            position_list.insert(0, current_arm_joint_pos_data)
            # print(position_list)
            # position_list = copy.deepcopy(position_list)
            step = len(position_list)
            for i in range(step - 1):
                print(current_arm_joint_pos_data)
                # print(i, ":", position_list[i])
                # print(i + 1, ":", position_list[i + 1])
                diff = np.sum(np.abs(position_list[i + 1] - position_list[i]))
                print(diff)
                duration = diff / 60
                if duration == 0:
                    continue
                # my_arm_move(position_list[i], position_list[i + 1], duration)
                my_arm_move(position_list[i], position_list[i + 1], 2)
    
        my_arm_continuous_move(action_list)
        return
    # Dex Hands


    def prepare(self):
        self.set_both_hands_action(self.ges_hand_release + self.ges_hand_release)
        '''Reset to prepare pose for table-top manipulation'''
        if self.prepared_flag:
            return
        gesture0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)

        lift_1 = np.array([75, 0, 0, -130, 0, 0, 0,    75, 0, 0, -130, 0, 0, 0], dtype=np.float64)
        lift_2 = np.array([15, 45, 0, -130, 0, 0, 0,    15, -45, 0, -130, 0, 0, 0], dtype=np.float64)
        lift_3 = np.array([-10, 20, -30, -100, -30, 0, 0,    -10, -20, 30, -100, 30, 0, 0], dtype=np.float64)
        basic_lift = [gesture0, lift_1, lift_2, lift_3]
        self.set_both_arm_traj(basic_lift)
        self.prepared_flag = True

    def set_left_hand_action(self, action_delta):
        
        raise NotImplementedError

    def set_right_hand_action(self, action_delta):
        raise NotImplementedError

    def set_both_hands_action(self, hand_joints):
        while not rospy.is_shutdown():

            both_hand_msg = dexhandCommand()

            # 示例：位置控制，各指关节中等关闭
            both_hand_msg.control_mode = 0  # POSITION_CONTROL

            # 左手：thumb, thumb_aux, index, middle, ring, pinky
            # right = hand_joints[:6]
            
            # # 右手顺序同上
            # left = hand_joints[-6:]

            # both_hand_msg.data = right + left # 一共 12 个元素

            both_hand_msg.data = hand_joints
            # rospy.loginfo(f"Publishing dexhandCommand: mode={both_hand_msg.control_mode}, data={both_hand_msg.data}")
            self.hand_pub.publish(both_hand_msg)

            if np.mean(np.array(self.get_current_positions()[1]) - np.array(hand_joints)) < 2:
                sleep(1)
                break
            rate = rospy.Rate(10)
            rate.sleep()


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
    # State retrival
    def get_current_positions(self):
        '''
        Return:
            1: 
            2: 
            3: ...
        '''
        global current_arm_joint_pos_data
        global current_hand_joint_pos_data
        return current_arm_joint_pos_data, current_hand_joint_pos_data
    
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
    # controller.demo_grab()
    # controller.set_both_arm_traj([np.array([-20, 10, -40, -90, -30, 0, 0,    -20, -10, 40, -90, 30, 0, 0], dtype=np.float64)]) #both arms
    # controller.demo_trans_between_hands()
    # TODO: a demo that enable a hard-code grasping
    controller.set_both_arm_traj([np.array([0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0], dtype=np.float64)])
    # controller.prepare()

if __name__ == "__main__":

    print('Run Kuavo demo test that grasp an object in the front table.', 'cyan')
    sleep(2)
    
    demo()
    print('demo test finished.', 'green')
