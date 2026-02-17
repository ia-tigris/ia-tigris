#!/usr/bin/env python
'''
Records rosparams at a low rate
'''
import yaml
import rospy
from datetime import datetime
from planner_map_interfaces.msg import PlanRequest

#names of ipp nodes
ipp_params = ['ipp_planners_node', 'realtime_search_map', 'sensor']

def plan_request_callback(data):
    filtered_params_dict = dict()
    # grab all parameters from rosparam server
    all_params = rospy.get_param_names()
    # filter params based on name
    filtered_params = [p for p in all_params
                       for ipp_p in ipp_params if ipp_p in p]
    #add param values to dictionary
    filtered_params_dict = {key: rospy.get_param(key) for key in filtered_params}
    #dump dictionary into a yaml
    curr_time =  datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    # curr_time = datetime.fromtimestamp(data.header.stamp.to_sec()).strftime("%Y-%m-%d %H:%M:%S")
    with open(rospy.get_param("~save_dir") + "/planner_params_" + curr_time + ".yaml", 'w') as file:
        yaml.dump(filtered_params_dict, file)
    rospy.loginfo("finished writing CMU params to file")

if __name__ == "__main__":
    rospy.init_node('param_recorder')
    rospy.Subscriber(rospy.get_param("~robot_name") + "/planner/plan_request", 
                        PlanRequest, plan_request_callback)
    rospy.spin()