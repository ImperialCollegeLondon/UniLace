#!/usr/bin/env python

import rospy
import json
import yaml
import rospkg
from uni_lace_msgs.srv import *


class UnityParamManager():
    def __init__(self, param=None):
        # load parameters from yaml file
        if param is None:
            param_file = "param"
            with open(rospkg.RosPack().get_path('uni_lace')+'/config/{}.yaml'.format(param_file)) as f:
                self.params = yaml.load(f, Loader=yaml.FullLoader)
        else:
            self.params = param

        self.params['use_gym'] = False
        # generate initial rope config (ROS coordinate space)
        start_pos = [0.25, self.params['rope_length']/2, self.params['rope_radius']+0.05]
        end_pos = [0.25, -self.params['rope_length']/2, self.params['rope_radius']+0.05]
        config = [start_pos]*self.params['rope_num_ctrl_pts'] # (num_particles*dim)
        pn = self.params['rope_num_ctrl_pts']
        for pi in range(pn):
            config[pi] = {"x": (pn-pi)*start_pos[0]/(pn-1) + pi*end_pos[0]/(pn-1),
                            "y": (pn-pi)*start_pos[1]/(pn-1) + pi*end_pos[1]/(pn-1),
                            "z": (pn-pi)*start_pos[2]/(pn-1) + pi*end_pos[2]/(pn-1)}
        self.params['rope_init_config'] = config

        # set up ros handle
        rospy.Service('unilace_param_service', UniLaceParamService, self.send_param_json)
        print('[UniLaceParam] Parameter service ready')

    def send_param_json(self, req):
        # convert self.params to json
        response = UniLaceParamServiceResponse()
        response.params_json.data = json.dumps(self.params)
        return response


if __name__ == '__main__' :
    rospy.init_node('unity_param_manager', anonymous=True)
    unity_param_manager = UnityParamManager()
    rospy.spin()
