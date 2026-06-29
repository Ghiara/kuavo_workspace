# kuavo_workspace
Controller for kuavo 4pro maxB

## 1. Hardware setup

### 1.1 Data collection using Quest 3 VR


For teleoperation with Quest 3, follow the [VR TELEOPERATION](teleoperation.md) to enable the VR control and data collection.


### 1.2 (Outdated) Enable script control of Kuavo 4 pro max B (in-house control script)

#### Downstream machine

[Downstream machine repository (kuavo-ros-opensource v1.3.3)](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/1.3.3/)

>[!NOTE]
> For downstream machine, all commands must be executed under `root`, except for python **data recording** script
> ```bash
> # enable root
> sudo su
> # exit root
> exit
> ```

In kuavo downstream machine, run under root to enable upper body control:
```bash

cd kuavo-ros-opensource-1.3.3

sudo su

source devel/setup.bash

# # run either 
# roslaunch humanoid_controllers load_kuavo_real.launch 

# or
roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch 
```


#### (Outdated) Upstream machine 

[Upstream machine code repository](https://gitee.com/leju-robot/kuavo_ros_application/tree/1.0.2)

2. copy paste the `controller.py` in `src` to upstream machine `kuavo_ros_application/src/`, then control the robot using python.


3. In upstream machine, run `controller.py` to control the robot using python script:

```bash

cd kuavo_ros_application

source devel/setup.bash

cd path/to/controller/folder

python3 controller.py

```

## 2. Model deployment using kuavo_data_challenge (dev branch)


### 2.1 Activate control
In down stream machine run:
```bash
cd ~/kuavo-ros-opensource-1.3.3 # Note that we use 1.3.3 to enable control

sudo su 

source devel/setup.bash

roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch 

# press `o` to activate control
```

### 2.2 Onboard deployment (Jetson)

In upstream machine, setup your configs in `~/kdc_ws/kuavo_data_challenge/`, upload your model with 1 rosbag at proper place (following the instruction of kuavo_data_challenge).

In upstream machine run:
```bash
cd ~/kdc_ws/kuavo_data_challenge

conda activate kdc

python kuavo_deploy/eval_kuavo.py

# press 2 (go to first step pose in the given rosbag) 
# then press 3 (model inference to run task)
```

### 2.3 (Recommand) SideMachine deployment

>[!NOTE]
> Every time restart/reconnect the robot with side machine. you need to setup the connection bridge again.

1. connect the cable from kuavo downstream USB port to side machine CAT port (10 Gbps) 

2. setup the bridge in downstream machine
```bash
# ssh connect the downstream machine (wireless)
# in downstream machine test:
nmcli connection show
# netplan-enx00e04c68345c  a7ed1639-39c7-3a69-b08f-b5809275228a  ethernet  enx00e04c68345c (kuavo-down -> kuavo-up)
# Wired Connection X xxxxxxxxxxxxxxx ethernet enxc8a362ab9999 (find your own side machine IFACE)

```
3. modify the IFACE in the `setup_bridge.sh`
```bash
# For our workstation(t7875)

# 桥接接口名称
BRIDGE=br0_t7875

# 物理网卡名称（替换成你的接口名）
IFACE1=enxc8a362ab9999
IFACE2=enx00e04c68345c
```


4. run bridging 
```bash
sudo chmod +x setup_bridge.sh
sudo bash setup_bridge.sh

nmcli connection show
# Then you will see something like following:
# br0_t7875                41ae24e4-70ab-4127-aca6-303af4270f79  bridge    br0_t7875 
# enxc8a362ab9999          a40f3b76-5a22-4e75-87dd-9caa5ee7f7a5  ethernet  enxc8a362ab9999 (PC t7875)
# netplan-enx00e04c68345c  a7ed1639-39c7-3a69-b08f-b5809275228a  ethernet  enx00e04c68345c (kuavo-up)

```

5. in side machine, make sure the wired connection is switched to `Kuavo` with ip `192.168.26.10`. make sure ROS_IP, ROS_MASTER_URI are setup in `.bashrc` accordingly:
```bash
export ROS_IP=192.168.26.10
export ROS_MASTER_URI=http://192.168.26.1:11311
```

6. ping and test connection
```bash
# In side machien
ping 192.168.26.12 # ping upstream machine
# In downstream machine 
ping 192.168.26.10 # ping side machine

# IP Address (For inference):
# Upstream machine 192.168.26.12
# Downstream machine 192.168.26.1 (ROS Master)
# Side machine (PC) 192.168.26.10

```

7. in side machine, run model inference using `kuavo_data_challenge`


## Optional Installation & Setup Instructions

### 1. Install H12pro teleoperation autostart node

If the H12 pro tele control is disabled, run following cmd to install the auto start node:
```bash

cd kuavo-ros-opensource/src/humanoid-control/h12pro_controller_node/scripts

sudo su

./deploy_autostart.sh

# In control plan, select 1.ocs2
```

### 2. Zero-Pose Calibration

Following this [instruction](https://kuavo.lejurobot.com/manual/basic_usage/kuavo-ros-control/docs/3%E8%B0%83%E8%AF%95%E6%95%99%E7%A8%8B/%E6%9C%BA%E5%99%A8%E4%BA%BA%E5%85%B3%E8%8A%82%E6%A0%87%E5%AE%9A/) to calibrate zero-pose when necessary. 

### 3. Switch the dex hand register information

In downstream machine:
```
sudo python3 ~/kuavo-ros-opensource/tools/check tool/Hardware tool.py

# press `o` to open developer tool

# press `j` to select tactile dex hand operation

# press press `1` to setup usb of dex hands

# type `yes` to switch the left-right hand mapping
```

### 4. Tactile perception test:

To get real-time sensor data plot and control signal data plot using GUI tool provided by BrainCo official.

In downstream machine, run:

```bash

./BrainCo_Touch_Hand_Test_Tool
```

- click Connect, and
  ************************************************************************

        ID: 1       PROTOCOL: MODBUS                              

        PORT: /dev/ttyUSB0       BAUDRATE: 115200                              

  ************************************************************************
- with Broadcast off




### 5. Tool-Chain for training and deployment

Following this [instruction](https://openlet.openatom.tech/explore/journalism/detail/562836764125958144) to see how to use `kuavo_data_challenge` to train ACT/DP and deploy them in the real-world test.