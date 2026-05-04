# Changelog

All notable changes to the Kuavo_down will be documented in this file. (`kuavo_opensource_1.3.3/`)

## 2026-05-04

### Settings modification
- **Expanded Upper Arm Workspace**: Increased the `upper_arm_cone_angle_forward` from 140° to 180° in the Quest3 IK configuration to allow for greater range of motion.
- **Optimized Teleoperation Cartesian Bounds**: Expanded the tracking volume for the Quest 3 HMD and controllers to prevent premature workspace limits.
  - `box_min_bound`: Adjusted from `[0.00, -0.6, -0.19]` to `[0.00, -0.8, -0.3]`
  - `box_max_bound`: Adjusted from `[0.4, 0.6, 0.4]` to `[0.8, 0.8, 0.4]`
- **Launch Configuration Refactoring**: Updated `launch_quest3_ik_videostream_robot_camera.launch` to support dynamic parameter injection.

### Code modification:
In`./src/manipulation_nodes/noitom_hi5_hand_udp_python/launch/launch_quest3_ik_videostream_robot_camera.launch`: 
- New lines added:

```xml
  <arg name="box_min_bound" default="[0.00, -0.8, -0.3]" /> 
  <arg name="box_max_bound" default="[0.8, 0.8, 0.4]" />
  
  <rosparam param="/quest3/box_min_bound" subst_value="True">$(arg box_min_bound)</rosparam>
  <rosparam param="/quest3/box_max_bound" subst_value="True">$(arg box_max_bound)</rosparam>
  <param name="/quest3/upper_arm_cone_angle_forward" value="180.0" />
```