# Teleoperation & data collection of kuavo 4pro using Quest3 VR + hand stick

## Official documentation

1. https://kuavo.lejurobot.com/manual/basic_usage/kuavo-ros-control/docs/2%E5%BF%AB%E9%80%9F%E5%BC%80%E5%A7%8B/VR%E6%93%8D%E4%BD%9C/5w_VR%E6%93%8D%E4%BD%9C/

2. https://openlet.openatom.tech/explore/journalism/detail/554977288647741440

## Setup

1. connection robot and quest3 in a common LAN (this is already done): 

```yaml

Account: TP-Link_5359_5G_YuanMeng
Password: 70686999

```
2. In Quest3, open and active `Kuavo-Hand-Track-MR` App.

3. start downstream machine control node (for our robot with serial number P4-690):

```bash

cd ~/kuavo-ros-opensource-1.3.3 # Note that we use 1.3.3 to enable control

sudo su 

source devel/setup.bash

roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch 

# press `o` to enable control

```

4. in downstream machine, run in another terminal:

- follow [this video](https://www.bilibili.com/video/BV1x7CgYWE8i/?spm_id_from=888.80997.embed_other.whitelist&t=9.334561&bvid=BV1x7CgYWE8i&vd_source=73f506b74bc2d678bb1316d63d1c2983) to check how to control the robot with VR hand joystick

- To configure head motion mode, modify the parameter in this json file: `kuavo-ros-opensource-1.3.3/src/manipulation_nodes/noitom_hi5_hand_udp_python/scripts/config.json` (Note: the `FIXED` mode now let robot head down towards table with target_yaw=25)

```bash

cd kuavo-ros-opensource-1.3.3

sudo su

source devel/setup.bash

# roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch \
#     ip_address:=192.168.1.115 \
#     use_cpp_incremental_ik:=true \
#     use_incremental_hand_orientation:=false

# Debug: enable video stream display in VR
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik_videostream_robot_camera.launch

# Debug: orbbec camera
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik_videostream_orbbec.launch

```
- Note: The Quest3 IP in our LAN is `192.168.1.115`, double check with `sudo arp-scan -I enp2s0 192.168.1.0/24`

5. take on the quest3, check if ping latency shown at left hand (if it shows, it indicates the connection is no problem)


> [!WARNING]
> following commandd may be dangerous when aligning the robot arms with your pose. Make sure there is sufficient space surrounding the robot when the first time you use the VR device, and make sure your current human arm pose stay close with robot arm pose.

6. **Initial unlock arms**: In Quest3, press two front triggers for 2 - 3 secs. This will enable arm pose alignment and release arm control. 

7. **Lock arm and back to Zero**: press `x` + `A` again will return to zero pose and lock arms.

8. **Unlock arms during operation**: press `x` + `A` again to enable arms alignment.

9. **Lock arms at current pose**: press `x` + `B` will lock the arms at current position.

10. **Dexhand operation**: place (not press) fingers on `x` and/or `A`, will control thumb auxiliary joints. press front triggers will control the dexhand grasp pose (all fingers).


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

