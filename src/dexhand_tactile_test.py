#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime
from termcolor import cprint
from controller import Controller

def dex_hand_tactile_test():
    # 1. 初始化
    controller = Controller(config={'test':0})
    rospy.loginfo('Resetting to init pose...')
    controller.reset()
    rospy.sleep(5)

    rospy.loginfo('Going to prepare pose...')
    controller.prepare()
    controller.sleep()

    # 2. 参数设置
    record_duration_sec = 20.0
    sample_hz = 20.0
    sample_rate = rospy.Rate(sample_hz)
    finger_names = ["thumb", "index", "middle", "ring", "pinky"]
    point_names = ["Point 1", "Point 2", "Point 3"]

    timestamps = []
    # 存储右手原始数据：维度将是 (N_samples, 5, 3)
    right_raw_hist = []

    rospy.loginfo(f"开始录制右手触觉数据 (3个采样点/指)，持续 {record_duration_sec}s...")
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        elapsed = now - start_time
        if elapsed > record_duration_sec:
            break

        state = controller.get_current_tactile_state()
        if state is None:
            sample_rate.sleep()
            continue

        # 仅获取右手数据 (5, 12)
        right_tactile = np.array(state["right_hand_tactile_state"], dtype=np.float64, copy=True)
        
        # 提取前 3 列（3个采样点的法向力）: 形状为 (5, 3)
        right_normal_forces = right_tactile[:, 0:3]

        timestamps.append(elapsed)
        right_raw_hist.append(right_normal_forces)
        sample_rate.sleep()

    # 3. 数据处理与转换
    if len(timestamps) == 0:
        rospy.logwarn("未录制到数据")
        return

    timestamps = np.asarray(timestamps)
    right_raw_hist = np.asarray(right_raw_hist)  # 形状变为 (N, 5, 3)

    # 4. 绘图 (5个子图)
    fig, axes = plt.subplots(5, 1, figsize=(12, 18), sharex=True)
    colors = ['tab:red', 'tab:green', 'tab:blue']

    for i in range(5):  # 遍历 5 根手指
        ax = axes[i]
        for p in range(3):  # 遍历每根手指的 3 个采样点
            ax.plot(timestamps, right_raw_hist[:, i, p], color=colors[p], label=point_names[p], linewidth=1.5)
        
        ax.set_ylabel(f"R-{finger_names[i]} Force (N)")
        ax.legend(loc='upper right', fontsize='small')
        ax.grid(True, linestyle="--", alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Right Hand: 3-Point Normal Forces per Finger", fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.97])

    # 5. 保存结果
    out_dir = os.path.dirname(os.path.abspath(__file__))
    ts_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    fig_path = os.path.join(out_dir, f"right_hand_3point_tactile_{ts_tag}.png")
    
    fig.savefig(fig_path, dpi=300)
    plt.close(fig)
    
    rospy.loginfo(f"测试完成，图表已保存至: {fig_path}")

if __name__ == "__main__":
    try:
        dex_hand_tactile_test()
    except rospy.ROSInterruptException:
        pass