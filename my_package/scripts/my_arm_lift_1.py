#!/usr/bin/env python

import numpy as np
import sys
sys.path.insert(0, '/home/lab/kuavo-ros-opensource/src/my_package/scripts')
from my_arm_move import my_arm_continuous_move, my_arm_to_origin

"""
  FOR a array long as 14: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
  1 & 8: whole_arm_front_backword_swinging joints: 
          left: + means backward and - means forward (-180 ~ 90)
          right: same (-180 ~ 90)
          
  2 & 9: whole_arm_outside_inside_swinging joints: 
          left: + means outside(open) and - means inside(close) (-20 ~ 120)
          right: reverse (-120 ~ 20)
          
  3 & 10: whole_arm_twisting joints: 
          left: + means outside(anti-clockwise) and - means inside(clockwise) (-90 ~ 90)
          right: + means inside(anti-clockwise) and - means outside(clockwise) (-90 ~ 90)
          
  4 & 11: elbow joints: 
          left: - means bending and no + (-150 ~ 0)
          right: same (-150 ~ 0)
          
  5 & 12: wrist_twisting joints:
          left: + means outside(anti-clockwise) and - means inside(clockwise) (-90 ~ 90) 
          right: + means inside(anti-clockwise) and - means outside(clockwise) (-90 ~ 90) 
          (same as whole_arm_twisting joints)

  6 & 13: wrist_outside_inside_swinging joints: 
          left: + means outside and - means inside (-75 ~ 40)
          right: reverse (-40 ~ 75)
        
  7 & 14: wrist_front_backword_swinging joints: 
          left: + means backward and - means forward (-40 ~ 40)
          right: same (-40 ~ 40)
"""
# Some written gestures
gesture0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)

lift_1 = np.array([75, 0, 0, -130, 0, 0, 0,    75, 0, 0, -130, 0, 0, 0], dtype=np.float64)
# lift_1_5 = np.array([65, 45, 0, -130, 0, 0, 0,    65, -45, 0, -130, 0, 0, 0], dtype=np.float64)
lift_2 = np.array([15, 45, 0, -130, 0, 0, 0,    15, -45, 0, -130, 0, 0, 0], dtype=np.float64)
# lift_2_5 = np.array([-10, 20, 0, -100, 0, 0, 0,    -10, -20, 0, -100, 0, 0, 0], dtype=np.float64)
lift_3 = np.array([-10, 20, -30, -100, -30, 0, 0,    -10, -20, 30, -100, 30, 0, 0], dtype=np.float64)
basic_lift = [gesture0, lift_1, lift_2, lift_3]

if __name__ == "__main__":
  # my_arm_continuous_move([gesture0])
  print(sys.version)
  # my_arm_continuous_move(basic_lift)
  # my_arm_continuous_move(basic_lift[::-1])