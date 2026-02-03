import rospy
from kuavo_msgs.msg import dexhandCommand

def dexhand_publisher():
    # 初始化 ROS 节点
    rospy.init_node('dexhand_command_publisher', anonymous=True)
    # 创建一个发布者，向 /dexhand/command 话题发送 dexhandCommand 类型消息
    pub = rospy.Publisher('/dexhand/command', dexhandCommand, queue_size=10)

    rate = rospy.Rate(10)

    # 这里举例：control_mode=0 表示 POSITION_CONTROL，1 表示 VELOCITY_CONTROL
    # data 数组长度必须 12，前 6 控制左手，后 6 控制右手
    # 位置控制时，0 完全打开，100 完全关闭
    # 速度控制时，-100 全开，+100 全关

    while not rospy.is_shutdown():
        msg = dexhandCommand()

        # 示例：位置控制，各指关节中等关闭
        msg.control_mode = 0  # POSITION_CONTROL

        # 左手：thumb, thumb_aux, index, middle, ring, pinky
        left = [50, 0, 50, 50, 50, 50]
        # 右手顺序同上
        right = [50, 0, 50, 50, 50, 50]

        msg.data = left + right  # 一共 12 个元素

        rospy.loginfo(f"Publishing dexhandCommand: mode={msg.control_mode}, data={msg.data}")
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        dexhand_publisher()
    except rospy.ROSInterruptException:
        pass