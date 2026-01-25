# Upper body base controller class
import numpy as np
from typing import Dict, Union, List, Tuple
from time import sleep
from termcolor import cprint


class Controller():
    '''
    Kuavo base controller that enable uppper body control of kuavo 4pro maxB
    Left arm: 7-DoF
    Right arm: 7-DoF
    Head: 2-DoF
    Left dex hand: 6-DoF
    Right dex hand: 6-DoF
    '''

    def __init__(self, config, *args, **kwargs):
        self.config = config
        self.current_joint_positions = np.zeros(28+6+6) # include dex hands, TODO: joint definition

    # Reset & prepare
    def reset(self):
        '''Reset to robot 0 initial position'''
        raise NotImplementedError   
    
    def prepare(self):
        '''Reset to prepare pose for table-top manipulation'''
        raise NotImplementedError

    # Head 
    def set_head_action(self, action_delta):
        raise NotImplementedError

    # Arms
    def set_left_arm_action(self, action_delta):
        raise NotImplementedError
    
    def set_right_arm_action(self, action_delta):
        raise NotImplementedError

    # Dex Hands
    def set_left_hand_action(self, action_delta):
        raise NotImplementedError

    def set_right_hand_action(self, action_delta):
        raise NotImplementedError

    # State retrival
    def get_current_joint_positions(self):
        '''
        Return:
            1: 
            2: 
            3: ...
        '''
        raise NotImplementedError
    

def demo():

    controller = Controller()
    # TODO: a demo that enable a hard-code grasping









if __name__ == "__main__":

    cprint('Run Kuavo demo test that grasp an object in the front table.', 'cyan')
    sleep(3)
    demo()
    print('demo test finished.', 'green')



    

