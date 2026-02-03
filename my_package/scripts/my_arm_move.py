#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
import numpy as np
import copy
import math

# 初始化ROS节点
rospy.init_node('my_arm_move')

# 创建服务代理
arm_traj_change_mode_client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)

# 创建请求对象
request = changeArmCtrlModeRequest()
request.control_mode = 2  # 设置控制模式

# 调用服务并获取响应
response = arm_traj_change_mode_client(request)

# 创建发布者
pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)

"""
    to get arm joints state
"""
def joint_pos_callback(msg):
    # 提取最后 14 个数字
    global current_arm_joint_pos_data
    current_arm_joint_pos_data = (np.array(msg.data[-14:]) * 57.3).round().astype(np.float64)
rospy.Subscriber('/humanoid_controller/optimizedState_mrt/joint_pos', Float64MultiArray, joint_pos_callback)

# 等待直到有订阅者连接
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)  # 适当的睡眠时间，避免 CPU 占用过高


"""
    to move arm
"""
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

        pub.publish(msg)  # 发布关节状态
        rospy.sleep(1.0 / rate)  # 按照发布频率睡眠

    # 最后一次发布，确保最终目标位置被发布
    msg.position = target_position.tolist()
    pub.publish(msg)

def my_arm_continuous_move(position_list):
    global current_arm_joint_pos_data
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
    return

def my_arm_to_origin():
    position_origin = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
    my_arm_continuous_move([position_origin])
    return

if __name__ == "__main__":
  # 这里调用封装好的函数进行机械臂的平滑运动
    position1 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)  # 示例current位置
    position2 = np.array([-30, 60, 0, -30, 0, -30, 30, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)  # 示例目标位置
    position3 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
    position4 = np.array([-20, 50, 0, -20, 0, -30, 30, 20, 0, 0, -30, 0, 0, 0], dtype=np.float64)  # 示例目标位置
    duration = 3  # 设置运动时间
    my_arm_continuous_move([position4, position1, position2, position3])