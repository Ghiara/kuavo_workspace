# Teleoperation & data collection of kuavo 4pro using Quest3 VR + hand stick

## Official documentation

1. https://kuavo.lejurobot.com/manual/basic_usage/kuavo-ros-control/docs/2%E5%BF%AB%E9%80%9F%E5%BC%80%E5%A7%8B/VR%E6%93%8D%E4%BD%9C/5w_VR%E6%93%8D%E4%BD%9C/

2. https://openlet.openatom.tech/explore/journalism/detail/562836764125958144


## Prerequisites

In upstream and downstream machine, make sure:

1. The camera autostart service in upstream machine is enabled:
```bash

# 上位机AGX终端执行 rs-enumerate-devices 查看左右手腕相机Device info/Serial Number
# 终端执行 sudo vim /etc/kuavo.conf 将查到的设备号改入 CAMERA_LEFT=,CAMERA_RIGHT=

sudo systemctl enable start_camera.service 

```

2. In downstream machine, the launch file (`kuavo-ros-opensource-1.3.3/src/manipulation_nodes/noitom_hi5_hand_udp_python/launch/launch_quest3_ik_videostream_robot_camera.launch`) is configured as follows:

make sure `ip_address`, `camera_publisher_name` is setup according to the robot individual hardware.

```html

<launch>
    <!-- 定义命令行参数 -->
    <arg name="send_srv" default="1"/> <!-- 1: 不需要手动打开手臂控制，0: 需要手动打开手臂控制 -->
    <arg name="version" default="4" />
    <arg name="ctrl_arm_idx" default="2" />
    <arg name="ik_type_idx" default="0" />
    <arg name="ip_address" default="192.168.1.121" />
    <arg name="control_torso" default="0" /> <!-- 0: do NOT control, 1: control torso  -->
    <!-- <arg name="camera_publisher_name" default="/camera/color" /> -->
    <arg name="camera_publisher_name" default="/cam_h/color" />
    <arg name="hand_reference_mode" default="thumb_index"/> <!-- fingertips, middle_finger, thumb_index -->

    <!-- motion_capture_ik -->
    <node pkg="motion_capture_ik" type="ik_ros_uni.py" name="ik_ros_uni" args=" --version $(arg version) --ctrl_arm_idx $(arg ctrl_arm_idx) --ik_type_idx $(arg ik_type_idx) --send_srv=$(arg send_srv) --control_torso=$(arg control_torso) --hand_reference_mode=$(arg hand_reference_mode)" output="screen">
    </node>

    <!-- noitom_hi5_hand_udp_python -->
    <node pkg="noitom_hi5_hand_udp_python" type="monitor_quest3.py" args="$(arg ip_address)" name="monitor_quest3" output="screen">
    </node>
    <node pkg="noitom_hi5_hand_udp_python" type="webrtc_videostream.py" name="webrtc_videostream" args="$(arg camera_publisher_name)" output="screen"/>
</launch>
```

3. (optional) In downstream machine, for cpp incremental control, make sure the launch file (`kuavo-ros-opensource-1.3.3/src/manipulation_nodes/noitom_hi5_hand_udp_python/launch/launch_quest3_ik.launch`) is set as follows:

make sure `ip_address`, `camera_publisher_name` is setup according to the robot individual hardware. and head camera display node in quest 3 is added.

```html

<launch>
    <!-- 定义命令行参数 -->
    <arg name="camera_publisher_name" default="/cam_h/color" />
    <arg name="ip_address" default="192.168.1.121"/>

    <arg name="version" default="4" />
    <arg name="ctrl_arm_idx" default="2" />
    <arg name="ik_type_idx" default="0" />
    <arg name="control_torso" default="false" /> <!-- false: do NOT control, true: control torso  -->
    <arg name="control_finger_type" default="0" /> <!-- 0: control all fingers by upper-gripper, 1: control thumb and index fingers by upper-gripper, control other fingers by lower-gripper  -->
    <arg name="predict_gesture" default="false" /> <!-- True or False -->
    <arg name="ee_type" default="qiangnao_touch"/>
    ...

<!-- Add camera video stream node  -->
    <node pkg="noitom_hi5_hand_udp_python" type="webrtc_videostream.py" name="webrtc_videostream" args="$(arg camera_publisher_name)" output="screen"/>
</launch>

```

4. The `Kuavo_Hand_Track_MR` app is installed in the Quest3 VR (https://kuavo.lejurobot.com/Quest_apks/leju_kuavo_hand-0.0.1-298-gdc7cfac.apk)

Installation instruction: `https://kuavo.lejurobot.com/manual/basic_usage/kuavo-ros-control/docs/5%E5%8A%9F%E8%83%BD%E6%A1%88%E4%BE%8B/%E9%80%9A%E7%94%A8%E6%A1%88%E4%BE%8B/VR%E4%BD%BF%E7%94%A8%E5%BC%80%E5%8F%91%E6%A1%88%E4%BE%8B/`


## Setup - use default python interface control (a bit jitter)

1. connection robot and quest3 in a common LAN (this is already done): 

```yaml

Account: TP-Link_5359_5G_YuanMeng
Password: 70686999

```
2. start downstream machine control node (for our robot with serial number P4-690):

```bash

cd ~/kuavo-ros-opensource-1.3.3 # Note that we use 1.3.3 to enable control

sudo su 

source devel/setup.bash

roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch 


# press `o` to enable control

```

3. in downstream machine, run in another terminal:

- follow [this video](https://www.bilibili.com/video/BV1x7CgYWE8i/?spm_id_from=888.80997.embed_other.whitelist&t=9.334561&bvid=BV1x7CgYWE8i&vd_source=73f506b74bc2d678bb1316d63d1c2983) to check how to control the robot with VR hand joystick

- To configure head motion mode, modify the parameter in this json file: `kuavo-ros-opensource-1.3.3/src/manipulation_nodes/noitom_hi5_hand_udp_python/scripts/config.json` (Note: the `FIXED` mode now let robot head down towards table with target_yaw=25)

```bash

cd kuavo-ros-opensource-1.3.3

sudo su

source devel/setup.bash

# Enable video stream in VR
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik_videostream_robot_camera.launch


```
- Note: The Quest3 IP in our LAN is `192.168.1.121`, double check with `sudo arp-scan -I enp2s0 192.168.1.0/24`


4. In Quest3, open and active `Kuavo-Hand-Track-MR` App.


5. In App, check if ping latency shown at left hand (if it shows, it indicates the connection is no problem)


> [!WARNING]
> following commandd may be dangerous when aligning the robot arms with your pose. Make sure there is sufficient space surrounding the robot when the first time you use the VR device, and make sure your current human arm pose stay close with robot arm pose.

6. **Initial unlock arms**: In Quest3, press two front triggers for 2 - 3 secs. This will enable arm pose alignment and release arm control. 

7. **Lock arm and back to Zero**: press `x` + `A` again will return to zero pose and lock arms.

8. **Unlock arms during operation**: press `x` + `A` again to enable arms alignment.

9. **Lock arms at current pose**: press `x` + `B` will lock the arms at current position.

10. **Dexhand operation**: place (not press) fingers on `x` and/or `A`, will control thumb auxiliary joints. press front triggers will control the dexhand grasp pose (all fingers).



## (option) Use cpp incremental control (more smooth actions)


1. In the terminal, replace above step `3` by running following cmd:
```bash

cd kuavo-ros-opensource-1.3.3

sudo su

source devel/setup.bash

# # None video stream in VR
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch \
    ip_address:=192.168.1.121 \
    use_cpp_incremental_ik:=true \
    use_incremental_hand_orientation:=false

```

> [!WARNING]
> following commandd may be dangerous when aligning the robot arms with your pose. Make sure there is sufficient space surrounding the robot when the first time you use the VR device, and make sure your current human arm pose stay close with robot arm pose.

2. **switch arm control mode to 2**: press `x` + `A` **twice** to enable arm control mode switched to **`2`**. (in terminal you will see the arm control mode changed from 0 -> 1 -> 2)

3. **control the robot arms using hand joysticks**: press the both side trigger, meanwhile you can use the joystick to enable the arm tracking. other operation is same as above.



## (optional)VR Wired Connection Mode

### Kuavo Offical Help Documention
1. VR Use Case Development: https://kuavo.lejurobot.com/manual/basic_usage/kuavo-ros-control/docs/5%E5%8A%9F%E8%83%BD%E6%A1%88%E4%BE%8B/%E9%80%9A%E7%94%A8%E6%A1%88%E4%BE%8B/VR%E4%BD%BF%E7%94%A8%E5%BC%80%E5%8F%91%E6%A1%88%E4%BE%8B/#quest3-vr%E6%8E%A7%E5%88%B6
2. Wired VR using case: https://kuavo.lejurobot.com/manual/basic_usage/kuavo-ros-control/docs/6%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/%E6%9C%89%E7%BA%BFVR%E6%96%B9%E6%A1%88%E4%BD%BF%E7%94%A8%E6%8C%87%E5%8D%97/

### Current IP address allocation - Last updated on 2026-04-16

According to the latest network scan results, the current IP address allocation in the lab is as follows:

| Device Name                     | IP Address      | MAC Address         | Notes                                     |
| :------------------------------ | :-------------- | :------------------ | :---------------------------------------- |
| **Core Router**                 | `192.168.1.1`   | `7c:f1:7e:1f:53:59` | TP-Link Core Router                       |
| **Kuavo-down Computer (Wired)** | `192.168.1.120` | `c8:a3:62:ab:99:99` | Wired connection, fixed MAC               |
| **Kuavo-down Computer (Wi-Fi)** | `192.168.1.103` | `ac:82:47:d7:76:2a` | Intel NUC built-in Wi-Fi                  |
| **Meta Quest 3 (Wired)**        | `192.168.1.121` | `14:4f:d7:da:a4:30` | Type-C to Ethernet adapter (Shanghai B&A) |
| **Synology NAS**                | `192.168.1.117` | `00:11:32:ba:27:d6` | Lab storage server                        |
| **Realtek Fixed Device**        | `192.168.1.100` | `00:e0:4c:b9:4d:c0` | REALTEK chip fixed alive device           |
| **Ubuntu Backup IP (DHCP)**     | `192.168.1.102` | `f8:3d:c6:56:f9:49` | Hostname `ubuntu.local`, DHCP assigned    |

Note that IP address allocation is related to the router and connection method. It is recommended to test the IP address allocation again after changing the connection method.

Test methods:
```bash
sudo apt install arp-scan # Install arp-scan for Linux
brew install arp-scan  # Install arp-scan for MacOS

# Ensure you are in the same LAN as the robot
sudo arp-scan -I enp2s0 192.168.1.0/24

# Latency test
ping 192.168.1.121
```

### Use wired connection

```bash
cd kuavo-ros-opensource-1.3.3

sudo su

source devel/setup.bash

# python mode
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik_videostream_robot_camera.launch ip_address:=192.168.1.121

# cpp incremental control 
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch \
    ip_address:=192.168.1.121 \
    use_cpp_incremental_ik:=true \
    use_incremental_hand_orientation:=false
```


## Rosbag Data collection

In the downstream machine, run following script to record the rosbags:

```bash

cd kuavo-ros-opensource-1.3.3

source devel/setup.bash

# before you start recording, configure the save path and rosbag name when necessary
python3 src/demo/examples_code/record_data/rosbag_tool.py

# In the terminal select: `recording the rosbag` to start the recording
# press `ctrl + c` to and save the recording
```

The to be recorded topics can be configured in `kuavo-ros-opensource-1.3.3/src/demo/examples_code/record_data/record_topics.json`

