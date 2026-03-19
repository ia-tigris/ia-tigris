#!/usr/bin/python3
import numpy as np
from datetime import datetime
import os, rospy, rospkg, pickle, roslaunch, rosnode
from std_msgs.msg import Header, Float32, UInt32
from ipp_simple_sim.msg import Detections
from planner_map_interfaces.msg import PlanRequest, SensorConstraint, TargetPrior,GridPrior, Plan, FilteredTarget
import sys

r = rospkg.RosPack()

class ParameterSweepManager(object):
    def __init__(self, folder_path, extend_list, radius_list, prune_list, horizon_list):
        self.folder_path = folder_path
        self.extend_list = extend_list
        self.radius_list = radius_list
        self.prune_list = prune_list
        if len(horizon_list) == 0:
            self.horizon_list = [None]
        else:
            self.horizon_list = horizon_list
        self.name = 'uav1'
        self.desired_rate = 1.0

        self.launch_args = ['metrics', 'sim_mc_runs.launch','rviz:=true', 'include_cpu_mem_monitor:=true', 
                       'cpu_mem_csv_file:=' + self.folder_path +'cpu_mem_metrics.csv', 
                       'log_plan_metrics:=true', 'plan_metrics_csv_directory:='+ self.folder_path,
                       'mc_config:=param_sweep1.yaml', 
                       'robot_name:=' + self.name, 'sim:=false']

    def run_parameter_sweep(self):
        random_runs = True
        print("Starting sweeps")
        #run tests
        for extend_param in self.extend_list:
            for radius_param in self.radius_list:
                for prune_param in self.prune_list:
                    for horizon_param in self.horizon_list:

                        folder_name = "extend_" + str(extend_param) + "_radius_" + str(radius_param) + "_prune_" + str(prune_param)
                        if horizon_param != None:
                            folder_name += "_horizon_" + str(horizon_param)
                        os.makedirs(os.path.join(self.folder_path + folder_name), exist_ok=True)
                        temp_path = self.folder_path + folder_name + "/"
                        print("saving results to " + str(temp_path))
                        if random_runs:
                            self.launch_args = ['metrics', 'sim_mc_runs.launch','rviz:=true', 'include_cpu_mem_monitor:=true', 
                                'cpu_mem_csv_file:=' + temp_path +'cpu_mem_metrics.csv', 
                                'log_plan_metrics:=true', 'plan_metrics_csv_directory:='+ temp_path,
                                'mc_config:=mc_testing_search1_param_v29.yaml', 
                                'robot_name:=' + self.name, 'sim:=simple']
                        else:
                            self.launch_args = ['metrics', 'sim_mc_runs.launch','rviz:=true', 'include_cpu_mem_monitor:=true', 
                                'cpu_mem_csv_file:=' + temp_path +'cpu_mem_metrics.csv', 
                                'log_plan_metrics:=true', 'plan_metrics_csv_directory:='+ temp_path,
                                'mc_config:=mc_testing_search1_param_v29.yaml', 
                                'robot_name:=' + self.name, 'sim:=simple']
                            
                        #launch ipp nodes
                        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                        roslaunch.configure_logging(uuid)
                        launch_file = [(roslaunch.rlutil.resolve_launch_arguments(self.launch_args)[0], self.launch_args[2:])]
                        ipp_launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
                        ipp_launch.start()
                        rospy.sleep(1)

                        #overwrite parameters with the ones we want
                        rospy.set_param("/ipp_planners_node/extend_dist", extend_param)
                        rospy.set_param("/ipp_planners_node/extend_radius", radius_param)
                        rospy.set_param("/ipp_planners_node/prune_radius", prune_param)
                        #grab params to make sure we set them correctly
                        extend_set_param = rospy.get_param("/ipp_planners_node/extend_dist")
                        radius_set_param = rospy.get_param("/ipp_planners_node/extend_radius")
                        prune_set_param = rospy.get_param("/ipp_planners_node/prune_radius")
                        param_set_string = "Parameters set to: extend_dist: {}, extend_radius: {}, prune_radius: {}".format(extend_set_param, radius_set_param, prune_set_param)
                        if horizon_param != None:
                            rospy.set_param("/ipp_planners_node/use_plan_horizon", True)
                            rospy.set_param("/ipp_planners_node/plan_horizon_length", horizon_param)
                            horizon_bool_set_param = rospy.get_param("/ipp_planners_node/use_plan_horizon")
                            horizon_len_set_param = rospy.get_param("/ipp_planners_node/plan_horizon_length")
                            param_set_string += ", use_horizon_length: {}, horizon_length: {}".format(horizon_bool_set_param, horizon_len_set_param)
                        rospy.logwarn(param_set_string)

                        #send plan request
                        if not random_runs:
                            os.system("rosrun planner_map_interfaces pub_plan_request_from_yaml.py /home/moon/code/onr_ws/src/planner_map_interfaces/config/fixed-wing/plan_requests/feature_tests/param_sweep_request.yaml {} \"\"")
                        
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
    # args are folder_path, extend, radius, prune, horizon, (next planning time/budget/etc)
    # planning time next
    print("Running parameter sweep. Starting rosnode")
    rospy.init_node('param_sweep_metrics')
    if len(sys.argv) > 4:
        # set folder path explicitly if an empty string is fed in
        folder_path = sys.argv[1]
        if len(folder_path) == 0:
            folder_path = r.get_path("ipp_planners") + "/test_results/param_sweep_execute/"
        extend_list = sys.argv[2].split(',')
        extend_list = [eval(i) for i in extend_list]
        radius_list = sys.argv[3].split(',')
        radius_list = [eval(i) for i in radius_list]
        prune_list = sys.argv[4].split(',')
        prune_list = [eval(i) for i in prune_list]
        #optional arg to test horizon length
        horizon_list = []
        if len(sys.argv) > 5:
            horizon_list = sys.argv[5].split(',')
            horizon_list = [eval(i) for i in horizon_list]

    manager = ParameterSweepManager(folder_path, extend_list, radius_list, prune_list, horizon_list)
    manager.run_parameter_sweep()
    # rospy.spin()