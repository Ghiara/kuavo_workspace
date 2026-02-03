## about basic usage
  cd kuavo-ros-opensource
  sudo su
  source devel/setup.bash
  roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch
  rosrun my_package my_controller.py

## about files
  my_arm_move.py is about some basic function for arm traj controlling,
  my_get_arm_joints.py is about getting current joint angles from subscription,
  my_arm_lift_1.py is about basic lifting motion of arms avoiding crash into desk, which is the preparation before the logistic task.

## about the joints
### arm
FOR a array long as 14: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
  - 1 & 8: whole_arm_front_backword_swinging joints: 
          left: + means backward and - means forward (-180 ~ 90)
          right: same (-180 ~ 90)
          
  - 2 & 9: whole_arm_outside_inside_swinging joints: 
          left: + means outside(open) and - means inside(close) (-20 ~ 120)
          right: reverse (-120 ~ 20)
          
  - 3 & 10: whole_arm_twisting joints: 
          left: + means outside(anti-clockwise) and - means inside(clockwise) (-90 ~ 90)
          right: + means inside(anti-clockwise) and - means outside(clockwise) (-90 ~ 90)
          
  - 4 & 11: elbow joints: 
          left: - means bending and no + (-150 ~ 0)
          right: same (-150 ~ 0)
          
  - 5 & 12: wrist_twisting joints:
          left: + means outside(anti-clockwise) and - means inside(clockwise) (-90 ~ 90) 
          right: + means inside(anti-clockwise) and - means outside(clockwise) (-90 ~ 90) 
          (same as whole_arm_twisting joints)

  - 6 & 13: wrist_outside_inside_swinging joints: 
          left: + means outside and - means inside (-75 ~ 40)
          right: reverse (-40 ~ 75)
        
  - 7 & 14: wrist_front_backword_swinging joints: 
          left: + means backward and - means forward (-40 ~ 40)
          right: same (-40 ~ 40)

Some common used position of arm (only half):


### hand
FOR a array long as 12: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
- 1 ~ 6 are right hand and 7 ~ 12 are left hand