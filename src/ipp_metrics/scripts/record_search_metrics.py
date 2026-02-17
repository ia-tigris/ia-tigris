#!/usr/bin/env python3
import numpy as np
from datetime import datetime
import os, rospy, rospkg, pickle, roslaunch
from std_msgs.msg import Header, Float32, UInt32
from ipp_simple_sim.msg import Detections
from planner_map_interfaces.msg import PlanRequest, SensorConstraint, TargetPrior,GridPrior, Plan, FilteredTarget
from geometry_msgs.msg import Pose, Point32, Point, Quaternion, Vector3, Polygon

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('metrics')
sr_path = pkg_path + "/search_requests/"
tr_path = pkg_path + "/sim_test_results/"

class SearchMetricsManager(object):
    def __init__(self):
        self.name = rospy.get_param("~robot_name")
        self.desired_rate = 5.0
        self.received_plan = False
        self.waypoint_num = 0
        self.avg_entropy = []
        self.avg_probs = []
        self.target_hist = dict()
        self.plan = Plan()
        self.plan_sub = rospy.Subscriber(self.name + "/global_path", Plan, self.plan_callback)
        self.avg_entropy_sub = rospy.Subscriber(self.name + "/realtime_search/average_entropy", Float32, self.avg_entropy_callback)
        self.avg_probs_sub = rospy.Subscriber(self.name + "/realtime_search/average_prob", Float32, self.avg_probs_callback)
        self.ipp_pr_pub = rospy.Publisher(self.name + "/planner/plan_request", PlanRequest, queue_size=1)
        self.waypoin_num_sub = rospy.Subscriber(self.name + "/waypoint_num", UInt32, self.waypoint_num_callback)
        self.detections_sub = rospy.Subscriber(self.name + "/sensor_measurement", Detections, self.detections_callback)

        self.launch_args = ['ipp_planners', 'main.launch','rviz:=true', 'search:=true', 
                       'track:=false', 'sim:=simple', 'robot_name:=' + self.name]
    
    def load_search_request_info(self,file):
        with open(sr_path + file, 'rb') as handle:
            search_request_info = pickle.load(handle)
        return search_request_info
    
    def avg_entropy_callback(self, data):
        self.avg_entropy.append(round(data.data,4))
    
    def avg_probs_callback(self,data):
        self.avg_probs.append(round(data.data,4))
    
    def waypoint_num_callback(self, data):
        if self.received_plan:
            self.waypoint_num = data.data

    def plan_callback(self, data):
        self.plan = data
        self.received_plan = True
    
    def detections_callback(self, data):
        for idx in range(len(data.target_ids)):
            #grab id and pose of the target
            target_id = data.target_ids[idx]
            ship_pose = data.target_camera_vectors[idx]
            ship_pose_vec = [ship_pose.x, ship_pose.y, ship_pose.z]
            #add to dictionary
            if target_id not in self.target_hist:
                self.target_hist[target_id] = [ship_pose_vec]
            else:
                self.target_hist[target_id].append(ship_pose_vec)

    def gen_plan_request(self, search_request_info):
        width,height = search_request_info['map_width'], search_request_info['map_height']
        ipp_plan_request = PlanRequest()
        ipp_plan_request.start_pose = Pose(Point(0.0,0.0,110.0), Quaternion(0.0,0.0,0.0,1.0))
        ipp_plan_request.wind_speed = Vector3(0.0,0.0,0.0)
        ipp_plan_request.max_planning_time = 10.0
        ipp_plan_request.maximum_range = 500.0
        ipp_plan_request.desired_speed = 10.0
        search_map = [Point32(0,0,0), Point32(width,0,0), Point32(width,height,0), Point32(0,height,0)]
        ipp_plan_request.search_bounds = Polygon(points=search_map)
        ipp_plan_request.counter_detection_range = 50.0
        ipp_plan_request.min_loiter_radius = 0.0
        ipp_plan_request.sensor_constraints = SensorConstraint(Vector3(-2.0,-2.0,-2.0), 
        Vector3(-2.0,-2.0,-2.0), -2.0, -1.0, -2.0, -1.0)
        #iterate through grid priors
        grid_prior_map = search_request_info['search_map'].ravel()
        grid_prior_coords = search_request_info['grid_priors']
        target_priors = []
        #add grid priors
        for conf, coords in zip(grid_prior_map, grid_prior_coords):
            prior_points = [Point32(point[0],point[1],point[2]) for point in coords]
            grid_prior = GridPrior(header=Header(frame_id='local_enu'),grid_prior_type=1,
                            bounds=Polygon(points=prior_points), confidence=conf,
                            priority=1.0,sensor_model_id=0)
            target_priors.append(TargetPrior(grid_prior=grid_prior))
        #add target priors
        filtered_targets = search_request_info['target_poses']
        for target_pose in filtered_targets:
            target_msg = FilteredTarget()
            target_msg.x = target_pose[0]
            target_msg.y = target_pose[1]
            target_msg.xdot = 0.0
            target_msg.ydot = 0.0
            target_msg.sensor_model_id = 0
            target_priors.append(TargetPrior(target=target_msg))

        ipp_plan_request.target_priors = target_priors
        ipp_plan_request.clear_tree = True
        return ipp_plan_request

    def reset_test(self):
        self.received_plan = False
        self.waypoint_num = 0
        self.avg_entropy = []
        self.avg_probs = []
        self.plan = Plan()
        self.target_hist = dict()

    def run_search_tests(self):
        #filter out files that aren't pickles
        pr_files = [f for f in np.sort(os.listdir(sr_path)) if 'pickle' in f]
        #run tests
        for file in pr_files:
            #generate search request
            pr_info = self.load_search_request_info(file)
            search_request = self.gen_plan_request(pr_info)
            #launch ipp nodes
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch_file = [(roslaunch.rlutil.resolve_launch_arguments(self.launch_args)[0], self.launch_args[2:])]
            ipp_launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
            ipp_launch.start()
            rospy.sleep(3.0) #wait for launch to finish
            #overwrite parameters with the ones we want
            rospy.set_param("sim_manager_node/sample_additional_true_targets_from_search_prior", False)
            #send plan request
            self.ipp_pr_pub.publish(search_request)
            #wait for end of budget
            rate = rospy.Rate(self.desired_rate)
            try:
                while (self.waypoint_num == 0 or self.waypoint_num < len(self.plan.plan)) and not rospy.is_shutdown():
                    rate.sleep()
                rospy.loginfo("test complete")
                #write results to file
                curr_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "/"
                os.makedirs(os.path.dirname(tr_path + curr_time), exist_ok=True)
                np.savetxt(tr_path + curr_time + self.name + "_search_map_metrics.csv", 
                           np.column_stack((self.avg_entropy, self.avg_probs)),
                           delimiter=',', header="avg_entropy, avg_probs", comments='')
                with open(tr_path + curr_time + self.name + "_detections_metrics.pickle", 'wb') as handle:
                    pickle.dump(self.target_hist, handle, protocol=pickle.HIGHEST_PROTOCOL) #change to protocol=2 if using python2.7
                rospy.loginfo("saved results to file")
                #reset test metrics and flags
                self.reset_test()
            finally:
                ipp_launch.shutdown()
           
if __name__ == "__main__":
    rospy.init_node('search_metrics')
    manager = SearchMetricsManager()
    manager.run_search_tests()
    rospy.spin()