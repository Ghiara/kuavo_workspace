#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

# 回调函数，接收 joint_pos 数据
def joint_pos_callback(msg):
    # 提取最后 14 个数字
    joint_pos_data = (np.array(msg.data[-14:]) * 57.3).round().astype(int)
    
    # 打印提取出来的关节角度数据
    rospy.loginfo("Extracted joint angles: %s", joint_pos_data)

def joint_pos_listener():
    # 初始化 ROS 节点
    rospy.init_node('joint_pos_listener', anonymous=True)

    # 订阅话题 /humanoid_controller/optimizedState_mrt/joint_pos
    rospy.Subscriber('/humanoid_controller/optimizedState_mrt/joint_pos', Float64MultiArray, joint_pos_callback)

    # 保持节点运行，直到节点关闭
    rospy.spin()

if __name__ == '__main__':
    try:
        joint_pos_listener()
    except rospy.ROSInterruptException:
        pass
