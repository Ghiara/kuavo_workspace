#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os
from kuavo_msgs.msg import dexhandTouchState

# =================================================================
# 核心配置：MD5 伪装 (Zero-Footprint Personalization)
# 由于上下位机 ROS 版本或消息定义可能存在微小差异，如果运行报错提示 
# "MD5 sum mismatch"，请取消下面这行的注释，并改为报错信息中提示的 Actual 值。
# =================================================================
# dexhandTouchState._md5sum = "c9b154cca802a6580a2e018ad49b7c5b" 

class TactileDualMonitor:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('tactile_dual_monitor', anonymous=True)
        
        # 订阅触觉话题 (Topic: /dexhand/touch_state)
        self.sub = rospy.Subscriber("/dexhand/touch_state", dexhandTouchState, self.callback)
        
        # 清屏并显示初始化信息
        os.system('clear')
        print("\033[92m[INFO] 双指触觉实时监控脚本已启动...\033[0m")
        print("[监控对象] 左手拇指 (Thumb) & 左手食指 (Index)")
        print("[数据说明] 每根手指包含 3 个独立传感单元 (Point 1/2/3)")
        print("-" * 85)
        # 预留输出空间
        print("\n" * 9) 

    def parse_finger_data(self, finger_msg):
        """解析单根手指的 3 个受力点数据"""
        data = []
        # 遍历该手指的 3 个受力点（法向/切向力通常需除以 100 转换为标准单位 N）
        # 对应消息定义中的 normal_force1/2/3, tangential_force1/2/3 等
        pts = [
            {"n": finger_msg.normal_force1/100.0, "t": finger_msg.normal_force1/100.0, "d": finger_msg.tangential_direction1},
            {"n": finger_msg.normal_force2/100.0, "t": finger_msg.normal_force2/100.0, "d": finger_msg.tangential_direction2},
            {"n": finger_msg.normal_force3/100.0, "t": finger_msg.normal_force3/100.0, "d": finger_msg.tangential_direction3}
        ]
        return pts

    def callback(self, msg):
        # 提取左手拇指 (index 0) 和 食指 (index 1)
        # 对应 kuavo_msgs/dexhandTouchState 中的 left_hand 数组
        thumb_data = self.parse_finger_data(msg.left_hand[0])
        index_data = self.parse_finger_data(msg.left_hand[1])

        # 构造输出字符串 (使用 ANSI 颜色码)
        # \033[1m 加粗, \033[93m 黄色(法向), \033[96m 青色(切向), \033[95m 紫色(角度)
        output = "\033[10F"  # 光标上移 10 行实现原地刷新
        output += "\033[1;32m--- 实时触觉双指监控 (左手) ---\033[0m\n"
        
        # 打印拇指数据
        output += "\033[1;34m[ 拇指 Thumb ]\033[0m\n"
        for i, p in enumerate(thumb_data):
            output += f" 点{i+1} | 法向: \033[93m{p['n']:5.2f}N\033[0m | 切向: \033[96m{p['t']:5.2f}N\033[0m | 方向: \033[95m{p['d']:5.1f}°\033[0m\033[K\n"
        
        output += "-" * 50 + "\033[K\n"
        
        # 打印食指数据
        output += "\033[1;34m[ 食指 Index ]\033[0m\n"
        for i, p in enumerate(index_data):
            output += f" 点{i+1} | 法向: \033[93m{p['n']:5.2f}N\033[0m | 切向: \033[96m{p['t']:5.2f}N\033[0m | 方向: \033[95m{p['d']:5.1f}°\033[0m\033[K\n"

        sys.stdout.write(output)
        sys.stdout.flush()

if __name__ == '__main__':
    try:
        monitor = TactileDualMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"\n[ERROR] 脚本发生错误: {e}")