# kuavo_workspace
Controller for kuavo 4pro maxB

## 1. Hardware setup


### 1.1 Enable script control of Kuavo 4 pro max B

#### Downstream machine

[Downstream machine code repository](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/1.3.3/)

>[!NOTE]
> For downstream machine, all commands must be executed under `root`
> ```bash
> # enable root
> sudo su
> # exit root
> exit
> ```

In kuavo downstream machine, run under root to enable upper body control:
```bash

cd kuavo-ros-opensource

sudo su

source devel/setup.bash

# # run either 
# roslaunch humanoid_controllers load_kuavo_real.launch 

# or
roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch 
```


#### Upstream machine 

[Upstream machine code repository](https://gitee.com/leju-robot/kuavo_ros_application/tree/1.0.2)

2. copy paste the `controller.py` in `src` to upstream machine `kuavo_ros_application/src/`, then control the robot using python.


3. In upstream machine, run `controller.py` to control the robot using python script:

```bash

cd kuavo_ros_application

source devel/setup.bash

cd path/to/controller/folder

python3 controller.py

```

### 1.2 Data collection using Quest 3 VR


For teleoperation with Quest 3, follow the [instruction of teleoperation](teleoperation.md) to enable the VR control and data collection using rosbags.




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

### 2.2 model deployment & inference

In upstream machine, setup your configs in `~/kdc_ws/kuavo_data_challenge/`, upload your model with 1 rosbag at proper place (following the instruction of kuavo_data_challenge).

In upstream machine run:
```bash
cd ~/kdc_ws/kuavo_data_challenge

conda activate kdc

python kuavo_deploy/eval_kuavo.py

# press 2 (go to first step pose in the given rosbag) 
# then press 3 (model inference to run task)
```



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