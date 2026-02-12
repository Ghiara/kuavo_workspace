# kuavo_workspace
Controller for kuavo 4pro maxB

## 1. Hardware setup


### 1.1 Enable script control of Kuavo 4 pro max B

1. In kuavo downstream machine, run under root:

```bash

cd kuavo-ros-opensource

sudo su

source devel/setup.bash

# # run either 
# roslaunch humanoid_controllers load_kuavo_real.launch 

# or
roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch 
```

2. copy paste the `controller.py` in `src` to upstream machine `kuavo_ros_application/src/`, then control the robot using python.

3. In kuavo upstream machine, run:

```bash

cd kuavo_ros_application

source devel/setup.bash

cd src/my_controller

python controller.py

```


### 1.2 (Optional) Install H12pro teleoperation autostart node

If the H12 pro tele control is disabled, run following cmd to install the auto start node:
```bash

cd kuavo-ros-opensource/src/humanoid-control/h12pro_controller_node/script

sudo su

./deploy_autostart.sh

# In control plan, select 1.ocs2
```


## Note

1. Do not recompile the workspace using something like `catkin build` or `catkin clean`.
