#!/usr/bin/env python

import subprocess
import cv2
import yaml
import numpy as np

import rospy
np.float = np.float64
import rospkg
import tf
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped
from uni_lace_msgs.srv import *

from uni_lace.unity_param_manager import UnityParamManager


class UniLaceLive():
    def __init__(self, param_file=None, render_mode=None):
        # load parameters from yaml file
        if param_file is None:
            print('No param file specified, using default')
            param_file = rospkg.RosPack().get_path('uni_lace')+'/config/param_default.yaml'
        with open(param_file) as f:
            self.params = yaml.load(f, Loader=yaml.FullLoader)

        # initialise param manager
        self.param_manager = UnityParamManager(self.params)

        # load the camera frames
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        tf_msgs = []
        for camera in self.params['cameras']:
            cam_frame = camera['name']+'_color_optical_frame'
            parent_frame = camera['parent']
            # publish static camera transforms
            tf_msgs.append(self.add_static_transform(parent_frame, cam_frame))
        self.broadcaster.sendTransform(tf_msgs)

        # launch the environment
        self.unity_process = subprocess.Popen(["/UniLace/UniLace/Build/UniLaceLive.x86_64"],
                                              stdout=subprocess.PIPE,
                                              stderr=subprocess.STDOUT,)

    def add_static_transform(self, parent_frame, camera_frame):
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = parent_frame
        static_transformStamped.child_frame_id = camera_frame

        # static_transformStamped.transform.translation = Vector3(0,0,0)
        static_transformStamped.transform.rotation = Quaternion(*tf.transformations.quaternion_from_euler(-1.5708, 0, -1.5708))
        return static_transformStamped
        
if __name__ == '__main__':
    rospy.init_node('uni_lace_live', anonymous=True)
    param_file = '/UniLace/param_live.yaml'
    env = UniLaceLive(param_file)
    rospy.spin()