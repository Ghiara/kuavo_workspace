# Perception class
import numpy as np
from typing import List, Tuple, Dict
from termcolor import cprint





class KuavoCamera():

    def __init__(self, use_wrist_cameras:bool=True, use_lidar:bool=False):

        self.use_wrist_cameras = use_wrist_cameras
        self.use_lidar = use_lidar
        
        self.head_stereo_cam = None
        self.left_wrist_cam = None
        self.right_wrist_cam = None
        self.lidar_cam = None

        if use_wrist_cameras:
            pass

        if use_lidar:
            raise NotImplementedError
    

    # TODO: color depth alignment??
    def get_depth_frame(self):
        raise NotImplementedError
    
    def get_color_frame(self):
        raise NotImplementedError