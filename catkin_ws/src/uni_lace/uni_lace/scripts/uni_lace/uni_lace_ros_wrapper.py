#!/usr/bin/env python

import sys
import yaml
import json
import numpy as np

import rospy
import roslaunch
np.float = np.float64
import rospkg
from std_msgs.msg import String
from uni_lace_msgs.srv import *

from uni_lace.unity_param_manager import UnityParamManager


class UniLaceRosWrapper():
    
    def __init__(self, params=None, debug=False):
        self.debug = debug
        print('[UniLaceRos] Initialising ...')
        # launch ros tcp endpoint
        print('[UniLaceRos] Launching ros_tcp_endpoint')
        sys.stdout = open('redirect.out','w') # suppress output
        # sys.stderr = open('redirect.err','w')
        uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
        roslaunch.configure_logging(uuid)
        cli_args = ['/catkin_ws/src/uni_lace/uni_lace/launch/uni_lace_gym.launch', '']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=roslaunch_file, is_core=True)
        self.launch.start()
        sys.stdout = sys.__stdout__ # restore output
        # sys.stderr = sys.__stderr__

        # initialise ros node for the wrapper
        rospy.init_node('uni_lace_ros_wrapper', anonymous=True)

        # get param_file as a ros parameter
        if params is None:
            # load parameters from yaml file
            with open(rospkg.RosPack().get_path('uni_lace')+'/config/param.yaml') as f:
                self.params = yaml.load(f, Loader=yaml.FullLoader)
        else:
            self.params = params

        # initialise param manager
        self.param_manager = UnityParamManager(self.params)

        # instantiate step client
        # rospy.wait_for_service('unity_step')
        self.step_client = rospy.ServiceProxy('unity_step', UniLaceStepService)
        # rospy.wait_for_service('unity_reset')
        self.reset_client = rospy.ServiceProxy('unity_reset', UniLaceResetService)
        print('[UniLaceRos] Service clients ready')
        # rospy.spin()

    def wait_for_service(self):
        rospy.wait_for_service('unity_step')
        rospy.wait_for_service('unity_reset')
    
    def shutdown(self):
        print("[UniLaceRos] Shutting down ROS wrapper")
        sys.stdout = sys.__stdout__
        # self.ros_core.terminate()
        # self.launch.shutdown()

    def ready(self):
        return not rospy.is_shutdown()

    def reset(self):
        request_msg = UniLaceResetServiceRequest()
        response = self.reset_client(request_msg)
        obs = self.process_obs(response)
        return obs

    def step(self, action):
        request_msg = UniLaceStepServiceRequest()
        request_msg.act = String(json.dumps(action))
        response = self.step_client(request_msg)
        obs = self.process_obs(response)
        return obs

    def process_obs(self, response):
        obs = json.loads(response.obs.data)
        for img in obs['rgbd_images']:
            if img['raw_range_rgb']['x'] == img['raw_range_rgb']['y']:
                img['rgb'] = None
            else:
                img['rgb'] = self.rgb_bytes_to_cv2(
                    response.obs_raw.data[img['raw_range_rgb']['x']:img['raw_range_rgb']['y']], 
                    img['resolution'])
            if img['raw_range_depth']['x'] == img['raw_range_depth']['y']:
                img['depth'] = None
            else:
                img['depth'] = self.depth_bytes_to_cv2(
                    response.obs_raw.data[img['raw_range_depth']['x']:img['raw_range_depth']['y']],
                    img['resolution'])
        return obs

    def rgb_bytes_to_cv2(self, byte_img, res):
        reshaped = np.frombuffer(byte_img, dtype=np.uint8).reshape((res['y'], res['x'], 3))
        return np.flipud(reshaped)

    def depth_bytes_to_cv2(self, byte_img, res):
        if len(byte_img) <= 1:
            return None
        depth = np.frombuffer(byte_img, dtype=np.float32).reshape((res['y'], res['x']))
        depth = np.flipud(depth)*1000
        return depth
        
if __name__ == '__main__':
    env = UniLaceRosWrapper()