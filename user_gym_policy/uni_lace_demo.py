#!/usr/bin/env python

import time
import yaml
import subprocess
import numpy as np

np.float = np.float64

import gymnasium as gym

from uni_lace.uni_lace_ros_wrapper import UniLaceRosWrapper

if __name__ == '__main__':
    # initialise the ROS wrapper
    with open('/UniLace/user_gym_policy/param_demo.yaml') as f:
        params = yaml.load(f, Loader=yaml.FullLoader)
    ros_wrapper = UniLaceRosWrapper(params, debug=False)
    print('[UniLaceDemo] ROS Wrapper ready')

    # launch the environment
    unity_process = subprocess.Popen(["/UniLace/UniLace/Build/Demo.x86_64"],
                                            stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT,)
    print('[UniLaceDemo] Unity ready')

    # wait for ctrl c
    try:
        while ros_wrapper.ready():
            time.sleep(1)
    except KeyboardInterrupt:
        print('[UniLaceDemo] Shutting down')
        unity_process.terminate()
        ros_wrapper.shutdown()
        print('[UniLaceDemo] Done')
        exit(0)

