## (Outdated) Enable script control of Kuavo 4 pro max B (in-house control script)

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
