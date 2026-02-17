#!/usr/bin/env python3

import numpy as np
import pickle, os, yaml, rospkg

from std_msgs.msg import Header, Float32, UInt32
from planner_map_interfaces.msg import PlanRequest, SensorConstraint, TargetPrior,GridPrior, Plan, FilteredTarget
from geometry_msgs.msg import Pose, Point32, Point, Quaternion, Vector3, Polygon

curr_dir = os.getcwd()
sr_path = curr_dir + "/../search_requests/"
# load parameters
os.makedirs(os.path.dirname(sr_path), exist_ok=True)
with open(curr_dir + "/../config/gen_plan_request.yaml", "r") as stream:
    params = yaml.safe_load(stream)

r = rospkg.RosPack()
curr_dir = os.getcwd()
sr_path = r.get_path("metrics") + "/search_requests/"
pr_path = r.get_path("planner_map_interfaces") + params['plan_request_path']

def load_search_request_info(file):
    with open(sr_path + file, 'rb') as handle:
        search_request_info = pickle.load(handle)
    return search_request_info

def gen_grid_prior_dict(points, conf):
    x_offset, y_offset = params['x_offset'], params['y_offset']
    points_dict = [{'x': float(point[0]+x_offset), 'y': float(point[1]+y_offset), 
                    'z': float(point[2])} for point in points]
    return {'grid_prior':{'header':{'frame_id': 'local_enu'},
                          'grid_prior_type': 1,
                          'bounds':{'points':points_dict},
                          'confidence': float(conf),
                          'priority': 1.0,
                          'sensor_model_id': 0}}

def gen_filtered_target_dict(target_pose, id):
    x_offset, y_offset = params['x_offset'], params['y_offset']
    return {'target':{'header':{'frame_id': 'local_enu'},
                      'local_id': id,
                      'global_id': id,
                      'x': float(target_pose[0]+x_offset),
                      'y': float(target_pose[1]+y_offset),
                      'xdot': 0.0,
                      'ydot': 0.0,
                      'covariance': [0.0,0.0,0.0,0.0,
                                     0.0,0.0,0.0,0.0,
                                     0.0,0.0,0.0,0.0,
                                     0.0,0.0,0.0,0.0],
                       'priority': 1.0,
                       'sensor_model_id': 0}}

def gen_filtered_target_save_dict(target_poses):
    filtered_targets_dict = {'targets': []}
    x_offset, y_offset = params['x_offset'], params['y_offset']
    for idx in range(len(target_poses)):
        target_pose = target_poses[idx]
        target_dict = {'x': float(target_pose[0]+x_offset),
                       'y': float(target_pose[1]+y_offset),
                       'x_dot': 0.0, 'y_dot': 0.0}
        filtered_targets_dict['targets'].append(target_dict)
    return filtered_targets_dict

def gen_plan_request(search_request_info):
    #convert plan request info into dictionary that we dump into a yaml
    plan_request_dict = dict()

    # planning parameters
    plan_request_dict['clear_tree'] = params['clear_tree']
    plan_request_dict['max_planning_time'] = params['max_planning_time']
    plan_request_dict['maximum_range'] = params['maximum_range']
    plan_request_dict['desired_speed'] = params['desired_speed']
    plan_request_dict['counter_detection_range'] = params['counter_detection_range']
    plan_request_dict['min_loiter_radius'] = params['min_loiter_rad']
    plan_request_dict['wind_speed'] = params['wind_speed']

    # agent pose
    plan_request_dict['start_pose'] = params['start_pose']
    plan_request_dict['current_speed'] = params['current_speed']

    # sensor params
    plan_request_dict['sensor_constraints'] = params['sensor_constraints']
    
    # search bounds
    width,height = search_request_info['map_width'], search_request_info['map_height']
    x_offset, y_offset = params['x_offset'], params['y_offset']
    search_map = [{'x':x_offset,'y':y_offset,'z':0}, {'x':width+x_offset,'y':y_offset,'z':0}, 
                  {'x':width+x_offset,'y':height+y_offset,'z':0}, {'x':x_offset,'y':height+y_offset,'z':0}]
    plan_request_dict['search_bounds'] = {'points': search_map}


    target_priors = []
    # add search priors to plan request
    if params['include_priors']:
        print("adding grid priors to plan request")
        grid_prior_map = search_request_info['search_map'].ravel()
        grid_prior_coords = search_request_info['grid_priors']
        # add grid priors
        for conf, coords in zip(grid_prior_map, grid_prior_coords):
            grid_prior_dict = gen_grid_prior_dict([[point[0],point[1],point[2]]for point in coords],conf)
            target_priors.append(grid_prior_dict)
    # add filtered targets to plan request
    if params['include_targets']:
        print("adding filtered targtes to plan request")
        filtered_targets = search_request_info['target_poses']
        for idx in range(len(filtered_targets)):
            filtered_target_dict = gen_filtered_target_dict(filtered_targets[idx], idx)
            target_priors.append(filtered_target_dict)
            
    # write target ground truth to external yaml
    if params['save_targets']:
        print("exporting filtered targets")
        filtered_targets_dict = gen_filtered_target_save_dict(search_request_info['target_poses'])
        with open(r.get_path(params['save_pkg']) + params['save_dir'], 'w') as ft_file:
            yaml.dump(filtered_targets_dict, ft_file, sort_keys=False)

    plan_request_dict['target_priors'] = target_priors

    return plan_request_dict

if __name__ == "__main__":
    # grab files in search request directory
    pr_files = [f for f in np.sort(os.listdir(sr_path)) if 'pickle' in f]
    # iterate through files and convert to plan requests
    for file in pr_files:
        pr_info = load_search_request_info(file)
        #turn info into search request
        sr = gen_plan_request(pr_info)
        # dump search request into yaml
        with open(pr_path + file.split('.')[0] + ".yaml", 'w') as sr_file:
             yaml.dump(sr, sr_file, sort_keys=False)

