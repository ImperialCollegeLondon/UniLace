#!/usr/bin/env python

import time
import subprocess
import numpy as np

np.float = np.float64

import gymnasium as gym

from uni_lace.uni_lace_ros_wrapper import UniLaceRosWrapper


class UniLaceGymEnv(gym.Env):
    '''
        Gym environment for UniLace
        - Action:
            arm_l_action: 7-dim list of np.array
            arm_r_action: 7-dim list of np.array
            gripper_l_action: 1-dim float in [0,1] (0: open, 1: close)
            gripper_r_action: 1-dim float in [0,1] (0: open, 1: close)
            arm_l_vel: float (1 for default speed)
            arm_r_vel: float (1 for default speed)
        - Observation:
            RGBDImage[] rgbd_images;
            float[] left_arm;
            float left_gripper;
            float[] right_arm;
            float right_gripper;
            GroundTruth ground_truth;
        - RGBDImage:
            rgb: resolution[0] x resolution[1] x 3 (np.array)
            depth: resolution[0] x resolution[1] (np.array)
            string name;
            Vector2Int raw_range_rgb;
            Vector2Int raw_range_depth;
            Vector2Int resolution;
            Pose camera_pose;
            float[] camera_intrinsics;
        - Ground truth:
            Pose aglet1;
            Pose aglet2;
            List<Pose> eyelets;
            float rope_tension1;
            float rope_tension2;
            float rope_tension;
            List<bool> eyelet_to_be_laced;
            bool success;
    '''

    debug = False
    _info = {}
    _reward = 0
    
    def __init__(self, params=None, debug=False, render_mode=None):
        self.debug = debug
        
        # initialise the ROS wrapper
        self.ros_wrapper = UniLaceRosWrapper(params, debug=self.debug)
        print('[UniLaceGym] ROS Wrapper ready')

        # launch the environment
        self.unity_process = subprocess.Popen(["/UniLace/UniLace/Build/UniLaceGym.x86_64"],
                                              stdout=subprocess.PIPE,
                                              stderr=subprocess.STDOUT,)
        self.ros_wrapper.wait_for_service()
        print('[UniLaceGym] Unity ready')
    
    def reset(self):
        obs = self.ros_wrapper.reset()
        reward = self.compute_reward()
        info = self._get_info()
        return obs, reward, False, info

    def step(self, arm_l_action, arm_r_action, gripper_l_action, gripper_r_action, arm_l_vel, arm_r_vel):
        # call step service
        action = {}
        action['left_arm'] = arm_l_action
        action['left_arm_vel'] = arm_l_vel
        action['left_gripper'] = gripper_l_action
        action['right_arm'] = arm_r_action
        action['right_arm_vel'] = arm_r_vel
        action['right_gripper'] = gripper_r_action
        # time the process
        total_start = time.time()
        obs = self.ros_wrapper.step(action)
        print('total time taken: ', time.time()-total_start)

        self._reward = self.compute_reward()
        self._info = obs['ground_truth']
        return obs, self._get_reward(), self._get_done(), self._get_info()

    def compute_reward(self):
        return 0
    
    def _get_reward(self):
        return self._reward
    
    def _get_info(self):
        return self._info
    
    def _get_done(self):
        to_be_laced = self._info['eyelet_to_be_laced']
        return all(to_be_laced)

    def close(self):
        # self.unity_process.terminate()
        # self.ros_wrapper.shutdown()
        return super().close()

    def __del__(self):
        print("[UniLaceGym] Shutting down")
        self.unity_process.terminate()
        self.ros_wrapper.shutdown()


if __name__ == '__main__':
    env = UniLaceGymEnv()
    obs, _, _, _ = env.step([], [], 0, 1, 1, 1)