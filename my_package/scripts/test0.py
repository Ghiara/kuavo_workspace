#!/usr/bin/env python

import rospy
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest

def main():
    # 初始化ROS节点
    rospy.init_node('arm_target_poses_publisher')

    # 创建服务代理
    arm_traj_change_mode_client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)

    # 创建请求对象
    request = changeArmCtrlModeRequest()
    request.control_mode = 2  # 设置控制模式

    try:
        # 调用服务并获取响应
        response = arm_traj_change_mode_client(request)
        rospy.loginfo("Control mode changed successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

    # 创建发布者
    pub = rospy.Publisher('/kuavo_arm_target_poses', armTargetPoses, queue_size=10)

    # 设置发布频率
    rate = rospy.Rate(10)  # 10 Hz

    # 创建消息对象
    msg = armTargetPoses()

    # 设置消息的初始数据
    msg.times = [3, 3, 3]  # 时间列表
    # msg.values = [0, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]  # 关节角度列表
    # msg.values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]  # 关节角度列表
    msg.values = [0, 0, 0, -15, 0, 0, 0, 10, 0, 0, -15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -15, 0, 0, 0, 10, 0, 0, -15, 0, 0, 0]  # 关节角度列表
    # 循环发布消息
    while not rospy.is_shutdown():
        # 发布消息
        pub.publish(msg)
        rospy.loginfo("Published arm target poses")

        # 按照频率暂停
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# import rospy 
# from kuavo_msgs.msg import armTargetPoses 
# from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse 

# # 初始化ROS节点 
# rospy.init_node('arm_target_poses_publisher') 

# # 创建服务代理 
# arm_traj_change_mode_client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode) 

# # 创建请求对象 
# request = changeArmCtrlModeRequest() 
# request.control_mode = 2 # 设置控制模式 

# # 调用服务并获取响应 
# response = arm_traj_change_mode_client(request) 
# # 创建发布者 
# pub = rospy.Publisher('/kuavo_arm_target_poses', armTargetPoses, queue_size=10) 

# # 创建消息对象 
# msg = armTargetPoses() 
# msg.times = [3, 4, 3]  # 时间列表
# # msg.values = [0, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]  # 关节角度列表
# # msg.values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]  # 关节角度列表
# msg.values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, -15, 0, 0, 0, 10, 0, 0, -15, 0, 0, 0]  # 关节角度列表

# # 发布消息 
# pub.publish(msg)