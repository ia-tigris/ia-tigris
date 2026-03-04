#!/usr/bin/env python3
import numpy as np
from datetime import datetime
import os, rospy, rospkg, pickle, roslaunch, rosnode
from std_msgs.msg import Header, Float32, UInt32
from ipp_simple_sim.msg import Detections
from planner_map_interfaces.msg import PlanRequest, SensorConstraint, TargetPrior,GridPrior, Plan, FilteredTarget
import sys


class MCManager(object):
    def __init__(self, folder_path):
        self.folder_path = folder_path
        self.name = 'uav1'
        self.desired_rate = 1.0

        self.launch_args = ['metrics', 'sim_mc_runs.launch','rviz:=true', 'include_cpu_mem_monitor:=true', 
                       'cpu_mem_csv_file:=' + self.folder_path +'cpu_mem_metrics.csv', 
                       'log_plan_metrics:=true', 'plan_metrics_csv_directory:='+ self.folder_path,
                       'mc_config:=param_sweep1.yaml', 
                       'robot_name:=' + self.name, 'sim:=false']

    def run_parameter_sweep(self):
        random_runs = True
        print("Starting mc testing with a plan request")
        #run tests


        self.launch_args = ['metrics', 'sim_mc_runs.launch','rviz:=false', 'include_cpu_mem_monitor:=true', 
            'cpu_mem_csv_file:=' + self.folder_path +'cpu_mem_metrics.csv', 
            'log_plan_metrics:=true', 'plan_metrics_csv_directory:='+ self.folder_path,
            'mc_config:=mc_testing_search1.yaml', 
            'robot_name:=' + self.name, 'sim:=simple']
        #launch ipp nodes
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = [(roslaunch.rlutil.resolve_launch_arguments(self.launch_args)[0], self.launch_args[2:])]
        ipp_launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
        ipp_launch.start()
        rospy.sleep(5)
        #send plan request
        os.system("rosrun planner_map_interfaces pub_plan_request_from_yaml.py /home/moon/code/onr_ws/src/planner_map_interfaces/config/research/plan_requests/benchmarks/search_scenario_1.yaml {} \"\"")
        rospy.loginfo("Sent plan request")
        #wait for end of budget
        rate = rospy.Rate(self.desired_rate)
        try:
            node_list = rosnode.get_node_names()
            fail_count = 0
            while ('/ipp_planners_node' in node_list and not rospy.is_shutdown()):
                if not rosnode.rosnode_ping('/ipp_planners_node', max_count=2):
                    fail_count += 1
                    if fail_count > 20:
                        rospy.logerr("Node /ipp_planners_node shows up but is not running. Shutting down.")
                        return
                rate.sleep()
                node_list = rosnode.get_node_names()
        finally:
            ipp_launch.shutdown()

        rospy.loginfo("Finished Testing")
           
if __name__ == "__main__":
    # args are folder_path, extend, radius, prune, (next planning time/budget/etc)
    # planning time next
    print("Running mc testing. Starting rosnode")
    rospy.init_node('mc_testing_node_with_plan_request')
    if len(sys.argv) > 1:
        folder_path = sys.argv[1]
    else:
        print("No folder path")
        sys.exit()

    manager = MCManager(folder_path)
    manager.run_parameter_sweep()
    # rospy.spin()