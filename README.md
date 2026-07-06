# kuavo_workspace
Controller for kuavo 4pro maxB

## 1. Data collection using Quest 3 VR

>[!IMPORTANT]
> For teleoperation with Quest 3, follow the [VR TELEOPERATION](teleoperation.md) to enable the VR control and data collection.


## 2. Model deployment using kuavo_data_challenge (dev branch)

### 2.1 Activate upper body control node
In down stream machine run:
```bash
cd ~/kuavo-ros-opensource-1.3.3 # Note that we use 1.3.3 to enable control

sudo su 

source devel/setup.bash

roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch 

# press `o` to activate control
```

### 2.2 (Default) SideMachine deployment

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
3. modify the IFACE in the [`setup_bridge.sh`](https://github.com/LejuRobotics/kuavo_data_challenge/blob/dev/kuavo_deploy/readme/setup_robot_connection.md), which should be positioned at downstream machine home path.
```bash
# For our workstation(t7875)

# 桥接接口名称
BRIDGE=br0_t7875

# 物理网卡名称（替换成你的接口名）
IFACE1=enxc8a362ab9999 # to t7875
IFACE2=enx00e04c68345c # to upstream
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

>[!NOTE]
> For this step, you only need to set once.

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

```bash
# press 2 (go to first step pose in the given rosbag) 
# then press 3 (model inference to run task)
```


### 2.2 (Optional) Onboard deployment (Jetson)

In upstream machine, setup your configs in `~/kdc_ws/kuavo_data_challenge/`, upload your model with 1 rosbag at proper place (following the instruction of kuavo_data_challenge).

In upstream machine run:
```bash
cd ~/kdc_ws/kuavo_data_challenge

conda activate kdc

python kuavo_deploy/eval_kuavo.py

# press 2 (go to first step pose in the given rosbag) 
# then press 3 (model inference to run task)
```


### 3. Tool-Chain for training and deployment

Following this [instruction](https://openlet.openatom.tech/explore/journalism/detail/562836764125958144) to see how to use `kuavo_data_challenge` to train ACT/DP and deploy them in the real-world test.