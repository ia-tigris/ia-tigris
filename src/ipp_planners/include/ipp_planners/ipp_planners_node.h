#ifndef ONRTIGRISNODE_H
#define ONRTIGRISNODE_H

// C++ std
#include <chrono>
#include <cmath>
#include <string>
#include <algorithm>

// ROS native
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>

// our custom messages
#include <planner_map_interfaces/Plan.h>
#include <planner_map_interfaces/PlanRequest.h>
#include <planner_map_interfaces/Waypoint.h>
#include "planner_map_interfaces/PlannerStatus.h"

// utils
#include "planner_map_interfaces/camera_projection.h"
#include "planner_map_interfaces/ros_utils.h"

// planners
#include "ipp_planners/Planner.h"
#include "ipp_planners/Tigris.h"
#include "ipp_planners/GreedySearchPlanner.h"
#include "ipp_planners/RandomSearchPlanner.h"
#include "ipp_planners/CoveragePlanner.h"
#include "ipp_planners/PrimTree.h"
#include "ipp_planners/PrimTreeBnB.h"
#include "ipp_planners/MCTSSearch.h"

// belief
#include "ipp_planners/InfoMap.h"
#include "ipp_planners/InfoMapSearch.h"

#include "ipp_planners/SearchMap.h"
#include "ipp_planners/SearchMapSetup.h"
#include "ipp_planners/SearchMapUpdate.h"

// visuals
#include "ipp_planners/visualizations.h"

using namespace ros_utils;

namespace ipp
{
    class PlannerNode
    {
    public:
        /* --- ROS SPECIFIC --- */
        ros::NodeHandle &nh;
        ros::NodeHandle &pnh;

        ros::Rate loop_rate;
        std::string local_frame = "local_enu";

        // subscribers
        ros::Subscriber plan_request_sub;     // mission parameters specified in plan request
        ros::Subscriber replan_sub;           // user asks to replan using the existing belief
        ros::Subscriber agent_state_sub;      // pose of the agent
        ros::Subscriber waypoint_num_sub;     // what waypoints have been reached
        ros::Subscriber remaining_budget_sub; // remaining budget from system (for replan at rate)

        // publishers
        ros::Publisher final_path_pub; // waypoints produced by the plan 

        /* --- MEMBER ATTRIBUTES --- */

        // agent state
        geometry_msgs::PoseStamped agent_pose;
        int waypoint_num = 0; // the current waypoint for the agent to reach
        int waypoint_num_before_replan = 0; // try to update waypoint_num during plan
        // int current_plan_num_waypoints = 0;

        // planning
        std::unique_ptr<Planner> ipp_planner;                    // pointer for polymorphism, unique_ptr for ownership
        planner_map_interfaces::PlanRequest latest_plan_req_msg; // input plan request
        planner_map_interfaces::Plan current_plan_msg;           // plan to publish
        std::vector<int> current_plan_waypoint_map;
        planner_map_interfaces::PlannerStatus planner_status_msg; // status of the planner

        bool first_plan_request_received = false;
        bool has_new_plan_request = false; // whether we got a new plan request message

        ros::Time time_of_last_replan;
        ros::Duration duration_between_replan;
        double remaining_budget = 0.0;
        double use_plan_horizon = false;
        double horizon_length = 0.0;

        // belief
        std::unique_ptr<InfoMap> info_map; // this is the latest info map from the real world. pointer permits polymorphism
        // visualization
        std::unique_ptr<PlannerVisualizer> visualizer;
        bool should_visualize;
        bool should_vis_while_planning;

        /**
         * @brief Construct a new Planning Node object
         *
         * @param nh
         */
        PlannerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh, bool no_init)
            : nh(nh),pnh(pnh),
              loop_rate(ros_utils::get_param<double>(pnh, "loop_rate")),
              should_visualize(ros_utils::get_param<bool>(pnh, "visualization")),
              should_vis_while_planning(ros_utils::get_param<bool>(pnh, "vis_while_planning"))
        {
            ROS_INFO("Use other constructor");
        }

        /**
         * @brief Construct a new Planning Node object
         *
         * @param nh
         */
        PlannerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
            : nh(nh),pnh(pnh),
              loop_rate(ros_utils::get_param<double>(pnh, "loop_rate")),
              should_visualize(ros_utils::get_param<bool>(pnh, "visualization")),
              should_vis_while_planning(ros_utils::get_param<bool>(pnh, "vis_while_planning"))
        {
            init_attributes();
            init_subscribers();
            init_publishers();
        }

        /**
         * @brief Main loop of the node.
         *
         */
        int run()
        {
            while (ros::ok())
            {
                planner_status_msg.planner_status = planner_map_interfaces::PlannerStatus::NODE_MAIN_LOOP;
                this->ipp_planner->planner_heartbeat_pub.publish(planner_status_msg);
                if (this->has_new_plan_request || this->is_time_to_replan())
                {
                    if (this->has_new_plan_request)
                    {
                        this->process_new_plan_request();
                    }
                    this->make_and_publish_plan();
                    this->time_of_last_replan = ros::Time::now();
                }
                ros::spinOnce();
                this->loop_rate.sleep();
            }
            return 0;
        }

        void init_attributes()
        {
            init_info_map();
            init_planner();
            init_visualizer();

            // Misc attributes
            double seconds_until_auto_replan = get_param<double>(pnh, "seconds_until_auto_replan");
            ROS_INFO("Duration between replans set to %f", seconds_until_auto_replan);
            this->duration_between_replan = ros::Duration(seconds_until_auto_replan);

            this->use_plan_horizon = get_param<bool>(pnh, "use_plan_horizon", false);
            if (this->use_plan_horizon)
            {
                this->horizon_length = get_param<double>(pnh, "plan_horizon_length");
            }
        }

        /**
         * @brief Chooses the info map based on the what's chosen in the ROS params
         */
        void init_info_map()
        {
            ROS_INFO("Search task");
            this->info_map = std::make_unique<ipp::InfoMapSearch>(this->nh, this->pnh);
        }

        /**
         * @brief Chooses the planner based on the what's chosen in the ROS params
         */
        void init_planner()
        {
            std::string planner_name;
            pnh.param<std::string>("planner", planner_name, "NONE");
            if (planner_name == "tigris")
            {
                this->ipp_planner = std::make_unique<ipp::Tigris>(this->nh, this->pnh);
            }
            else if (planner_name == "mcts_search")
            {
                this->ipp_planner = std::make_unique<ipp::MCTSSearch>(this->nh, this->pnh);
            }
            else if (planner_name == "greedy")
            {
                this->ipp_planner = std::make_unique<ipp::GreedySearchPlanner>(this->nh, this->pnh);
            }
            else if (planner_name == "random")
            {
                this->ipp_planner = std::make_unique<ipp::RandomSearchPlanner>(this->nh, this->pnh);
            }
            else if (planner_name == "primtree")
            {
                this->ipp_planner = std::make_unique<ipp::PrimTree>(this->nh, this->pnh);
            }
            else if (planner_name == "primtreebnb")
            {
                this->ipp_planner = std::make_unique<ipp::PrimTreeBnB>(this->nh, this->pnh);
            }
            else if (planner_name == "rect_coverage")
            {
                this->ipp_planner = std::make_unique<ipp::CoveragePlanner>(this->nh, this->pnh);
            }
            else
            {
                throw std::runtime_error("Invalid planner name: " + planner_name);
            }
        }

        void init_visualizer()
        {
            std::string planner_name;
            pnh.param<std::string>("planner", planner_name, "NONE");
            if (planner_name == "tigris")
            {
                this->visualizer = std::make_unique<TigrisVisualizer>(this->nh, this->pnh);
            }
            else if (planner_name == "mcts_search")
            {
                this->visualizer = std::make_unique<ipp::MCTSSearchVisualizer>(this->nh, this->pnh);
            }
            else if (planner_name == "greedy")
            {
                this->visualizer = std::make_unique<ipp::GreedySearchVisualizer>(this->nh, this->pnh);
            }
            else if (planner_name == "random")
            {
                this->visualizer = std::make_unique<ipp::RandomSearchVisualizer>(this->nh, this->pnh);
            }
            else if(planner_name == "primtree")
            {
                this->visualizer = std::make_unique<ipp::PrimTreeVisualizer>(this->nh, this->pnh);
            }
            else if(planner_name == "primtreebnb")
            {
                this->visualizer = std::make_unique<ipp::PrimTreeBnBVisualizer>(this->nh, this->pnh);
            }
            else if (planner_name == "rect_coverage")
            {
                this->visualizer = std::make_unique<ipp::CoverageVisualizer>(this->nh, this->pnh);
            }
            else
            {
                ROS_WARN_STREAM("No visualizer for planner " << planner_name);
                this->visualizer = std::unique_ptr<TigrisVisualizer>(nullptr);
            }
        }

        void init_subscribers()
        {
            // subscribes to new mission parameters
            this->plan_request_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "plan_request_topic"), 1, &PlannerNode::plan_request_callback, this);
            // subscribe to a user request to replan with the existing mission parameters and budget
            this->replan_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "replan_topic"), 1, &PlannerNode::replan_request_callback, this);
            // this->agent_state_sub = nh.subscribe(ros_utils::get_param<std::string>(nh, "agent_odom_topic"), 1, &PlannerNode::agent_state_callback, this);
            this->agent_state_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "agent_odom_topic"), 1, &PlannerNode::agent_odom_callback, this);
            this->remaining_budget_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "remaining_budget_topic"), 1, &PlannerNode::remaining_budget_callback, this);
            this->waypoint_num_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "waypoint_num_topic"), 1, &PlannerNode::waypoint_reached_callback, this);
        }

        void init_publishers()
        {
            this->final_path_pub = nh.advertise<planner_map_interfaces::Plan>(ros_utils::get_param<std::string>(pnh, "plan_output_topic"), 10);
        }

        /**
         * @brief Replan with current tree when received a REplan request message
         *
         * @param replan_msg
         */
        void replan_request_callback(const std_msgs::String &replan_msg)
        {
            ROS_INFO_STREAM("Replan request received with message: " << replan_msg);
            if (first_plan_request_received)
            {
                this->has_new_plan_request = true;
                this->latest_plan_req_msg.clear_tree = false; // assume that replanning means we don't want to force-clear the tree
            }
            else
            {
                ROS_ERROR("First planning request has not been received, so there are no mission parameters set. Please publish a PlanRequest.msg first.");
            }
        }

        /**
         * @brief Given a replan message, sets the planning request parameters and clears the current plan.
         *
         * @param msg
         */
        void plan_request_callback(const planner_map_interfaces::PlanRequest &msg)
        {
            planner_status_msg.planner_status = planner_map_interfaces::PlannerStatus::RECEIVED_PLAN_REQUEST;
            this->ipp_planner->planner_heartbeat_pub.publish(planner_status_msg);
            
            this->latest_plan_req_msg = msg; // copy out the message. this is cheap because it's just built-in types

            ROS_INFO_STREAM("\n\nReceived Replan Message of scenario number " << msg.scenario << "\n");

            this->first_plan_request_received = true;
            this->has_new_plan_request = true;
        }

        void process_new_plan_request()
        {
            ROS_INFO_STREAM("\n\nProcessed Replan Message");
            ROS_INFO_STREAM("Max Planning Time: " << latest_plan_req_msg.max_planning_time);
            ROS_INFO_STREAM("Counter Detection Range: " << latest_plan_req_msg.counter_detection_range);
            ROS_INFO_STREAM("Maximum Agent Range: " << latest_plan_req_msg.maximum_range);
            ROS_INFO_STREAM("Desired Speed: " << latest_plan_req_msg.desired_speed);
            
            this->current_plan_msg.plan.clear();
            this->current_plan_waypoint_map.clear();
            this->remaining_budget = latest_plan_req_msg.maximum_range;

            bool trigger_clear_tree = this->ipp_planner->save_plan_request_params(latest_plan_req_msg);
            this->info_map->save_plan_request_params(latest_plan_req_msg);
            this->info_map->send_plan_request_params_to_belief(latest_plan_req_msg, this->ipp_planner->flight_height);
            this->latest_plan_req_msg.clear_tree = this->latest_plan_req_msg.clear_tree || trigger_clear_tree;
        }

        // update the state of our agent
        void remaining_budget_callback(const std_msgs::Float32 &msg)
        {
            this->remaining_budget = msg.data;
        }

	void agent_odom_callback(const nav_msgs::Odometry &msg){
		geometry_msgs::PoseStamped pose;
		pose.header = msg.header;
		pose.pose = msg.pose.pose;
		this->agent_pose = pose;

		// // update which waypoint we're going to 
		// auto waypoints = this->current_plan_msg.plan;
		// if (waypoints.size()>0){
		// 	auto agent_x = agent_pose.pose.position.x;
		// 	auto agent_y = agent_pose.pose.position.y;
		// 	auto agent_z = agent_pose.pose.position.z;
		// 	auto waypoint_pose =  waypoints[this->waypoint_num].position;
        //     // ROS_DEBUG_STREAM("Waypoint to reach idx=" << this->waypoint_num << ", waypoint pose: " << waypoint_pose);
		// 	auto wpt_x = waypoint_pose.position.x;
		// 	auto wpt_y = waypoint_pose.position.y;
		// 	auto wpt_z = waypoint_pose.position.z;
        //     // ROS_DEBUG_STREAM("Agent pose: " << agent_pose.pose.position);
		// 	auto distance = std::sqrt((agent_x - wpt_x) * (agent_x-wpt_x) + (agent_y - wpt_y) * (agent_y-wpt_y) + (agent_z - wpt_z) * (agent_z-wpt_z));
        //     ROS_INFO_STREAM("Distance to waypoint " << waypoint_num << ": " << distance);
		// 	if (distance <= 10){
		// 		this->waypoint_num += 1;
		// 		ROS_INFO_STREAM("Auto incrementing target waypoint num to " << waypoint_num);
		// 	}
		// }
	}
        // update the state of our agent
        void agent_state_callback(const geometry_msgs::PoseStamped &msg)
        {
            this->agent_pose = msg;
            // ROS_INFO_STREAM("New agent pose at x::" << agent_pose.pose.position.x);
        }

        // subscribes to check what waypoint index has been reached
        void waypoint_reached_callback(const std_msgs::UInt32 &msg)
        {
            if (msg.data > this->waypoint_num)
            {
                ROS_INFO_STREAM("Waypoint " << (int)msg.data << " reached with budget remaining " << remaining_budget);
                // TODO: I (andrew) added this to see if we can remove the waypoints that are already reached in the visuals. not sure if it'll break something
            }
            this->waypoint_num = msg.data;
            // ROS_INFO_STREAM("Planner received target waypoint: " << msg.data);
        }
        

        /**
         * @brief Make a plan that follows the current Plan Request parameters.
         * If the Plan Request is fresh, create a completely fresh tree to plan from.
         * Else, continue planning from the old tree.
         * Publishes the plan to the ROS topic.
         */
        void make_and_publish_plan()
        {
            auto start_time = ompl::time::now();

            // when a new plan request comes in, see if tree was asked to be clear
            // otherwise, defer to the bool set in the config file
            bool should_force_clear_tree = (this->has_new_plan_request) ? this->latest_plan_req_msg.clear_tree : ros_utils::get_param<bool>(pnh, "clear_tree_on_auto_replan");
            ROS_INFO_STREAM("Should force clear tree? " << should_force_clear_tree ? "true" : "false");
            // bool should_force_clear_tree = true; //TODO!!!

            this->waypoint_num_before_replan = this->waypoint_num;
            ROS_DEBUG_STREAM("waypoint num before replan is " << this->waypoint_num_before_replan);
            std::vector<double> pose_to_plan_from = this->decide_pose_to_plan_from();
            if (pose_to_plan_from.size() == 0)
            {
                ROS_INFO("It passed back correctly and no plan will happen");
                return;
            }

            double planning_budget = this->calculate_planning_budget_and_prop_info_map();

            // only plan if we successfully fetched info. else we should spin
            if (!this->info_map->fetch_latest_belief(pose_to_plan_from, planning_budget))
            {
                planner_status_msg.planner_status = planner_map_interfaces::PlannerStatus::FAILED_TO_FETCH_BELIEF;
                this->ipp_planner->planner_heartbeat_pub.publish(planner_status_msg);
            }

            // Make the plan.
            if (pose_to_plan_from.size() != 0) // pose to plan from returns okay
                this->ipp_planner->replan<PlannerVisualizer>(*this->info_map, pose_to_plan_from, planning_budget, should_force_clear_tree, this->visualizer.get());

            double time_elapsed = duration_cast<duration<double>>(Clock::now() - start_time).count();

            // The received plan request is no longer new, because we used it just now
            this->current_plan_msg.scenario = this->latest_plan_req_msg.scenario;
            this->has_new_plan_request = false;

            // update waypoint reached after replan
            ros::spinOnce();
            ROS_DEBUG_STREAM("waypoint num after replan is " << this->waypoint_num);
            // Publish the plan
            // this->current_plan_msg = this->create_path_msg();
            // this->trim_old_path_at_front(time_elapsed);
            this->updated_trim_old_path_at_front();
            this->append_path_msg();
            planner_status_msg.planner_status = planner_map_interfaces::PlannerStatus::PUBLISHED_NEW_PLAN;
            this->ipp_planner->planner_heartbeat_pub.publish(planner_status_msg);
            this->final_path_pub.publish(this->current_plan_msg);
            // this->current_plan_num_waypoints = this->current_plan_msg.plan.size();
            ROS_INFO_STREAM("Published plan with " << this->current_plan_msg.plan.size() << " waypoints");
            this->waypoint_num = 0;

            // publish plan to visualizer
            if (this->should_visualize && this->current_plan_msg.plan.size() > 0)
            {
                this->visualize(pose_to_plan_from, current_plan_waypoint_map);
            }
        }

        /**
         * @brief if a plan exists, check which waypoint is next and return the pose of that waypoint. else return the pose of where the drone is now.
         */
        std::vector<double> decide_pose_to_plan_from()
        {
            ROS_INFO("Deciding pose to plan from");
            std::vector<double> pose_to_plan_from;
            if (this->current_plan_msg.plan.size() > 0)
            {
                ROS_INFO("Current plan not empty");
                // keep the next "big waypoint" in the current plan and start from there
                int waypoint_idx_keep = this->find_expected_waypoint_reached_after_time_to_plan();
                ROS_INFO_STREAM("Waypoint index keep: " << waypoint_idx_keep);
                // if (waypoint_idx_reached != -1) // out of budget or replan can't find waypoint close enough
                pose_to_plan_from = this->get_coords_of_waypoint_num(waypoint_idx_keep);
                // else
                // {
                //     ROS_INFO("Waypoint index -1");
                //     return pose_to_plan_from;
                // }
            }
            else
            {
                // drone hasn't ever moved, so plan from where the plan request wants us to plan from
                auto start_pose = this->latest_plan_req_msg.start_pose;
                tf::Quaternion q(
                    start_pose.orientation.x,
                    start_pose.orientation.y,
                    start_pose.orientation.z,
                    start_pose.orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                pose_to_plan_from = {start_pose.position.x, start_pose.position.y, start_pose.position.z, yaw};
            }
            ROS_INFO_STREAM("Planning from pose: " << pose_to_plan_from[0] << ", " << pose_to_plan_from[1] << ", " << pose_to_plan_from[2] << ", " << pose_to_plan_from[3]);
            return pose_to_plan_from;
        }

        /**
         * Gets the Euclidean distance between two ROS points.
         * TODO: move to a utils file
         * @return the distance
         */
        double xyz_distance(geometry_msgs::Point &p1, geometry_msgs::Point &p2)
        {
            return std::sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
        }

        /**
         * @brief Given a waypoint index from our current plan, get the XYZPsi coordinates
         *
         * @param waypoint_num
         * @return std::vector<double>
         */
        std::vector<double> get_coords_of_waypoint_num(int waypoint_idx_keep)
        {
            if (waypoint_idx_keep == -1)
            {
                ROS_INFO("Waypoint index -1, using last current plan msg");
                double roll, pitch, yaw;
                tf::Quaternion q(
                    current_plan_msg.plan.back().position.orientation.x,
                    current_plan_msg.plan.back().position.orientation.y,
                    current_plan_msg.plan.back().position.orientation.z,
                    current_plan_msg.plan.back().position.orientation.w);
                tf::Matrix3x3 m(q);
                m.getRPY(roll, pitch, yaw);
                return {this->current_plan_msg.plan.back().position.position.x,
                        this->current_plan_msg.plan.back().position.position.y,
                        this->current_plan_msg.plan.back().position.position.z,
                        yaw};
            }
            std::vector<double> coords(4);
            std::vector<og::PathGeometric> best_path_segments = this->ipp_planner->get_best_path_segments();
            ROS_INFO_STREAM("Getting coords of waypoint num " << waypoint_idx_keep - 1 << " out of " << best_path_segments.size());
            // we set the new root to the last waypoint in our plan
            // get the last setpoint in the path segment
            if (best_path_segments.size() == 0 || best_path_segments.size() < waypoint_idx_keep)
            {
                if (best_path_segments.size() == 0)
                    ROS_WARN("Path segment state count is 0. Using last current_plan_msg");
                else
                    ROS_WARN_STREAM("Path segment state count is " << best_path_segments.size() << " and waypoint index keep is " << waypoint_idx_keep - 1 << ". Using last current_plan_msg");
                double roll, pitch, yaw;
                tf::Quaternion q(
                    current_plan_msg.plan.back().position.orientation.x,
                    current_plan_msg.plan.back().position.orientation.y,
                    current_plan_msg.plan.back().position.orientation.z,
                    current_plan_msg.plan.back().position.orientation.w);
                tf::Matrix3x3 m(q);
                m.getRPY(roll, pitch, yaw);
                return {this->current_plan_msg.plan.back().position.position.x,
                        this->current_plan_msg.plan.back().position.position.y,
                        this->current_plan_msg.plan.back().position.position.z,
                        yaw};
            }
            auto &path_segment = best_path_segments.at(waypoint_idx_keep - 1);
            auto *state = path_segment.getState(0)->as<XYZPsiStateSpace::StateType>();
            coords[0] = state->getX();
            coords[1] = state->getY();
            coords[2] = state->getZ();
            coords[3] = state->getPsi();
            return coords;
        }

        /**
         * @brief Computes which waypoint we expect the drone to have bypassed by the time
         *  planning duration finishes.
         *
         * @return int index of the waypoint in the current plan
         */
        int find_expected_waypoint_reached_after_time_to_plan()
        {                                                                                                                          // Distance we can fly in the amount of time given to make a new plan
            double dist_traveled_while_planning = this->ipp_planner->desired_speed * (this->ipp_planner->max_planning_time + 1.5); /// MAGIC NUMBER!!! TODO
            ROS_DEBUG_STREAM("Dist traveled while planning: " << dist_traveled_while_planning << " because speed is " << this->ipp_planner->desired_speed << " and time is " << this->ipp_planner->max_planning_time);
            double dist_count = 0;
            int loop_waypoint = this->waypoint_num;
            ROS_DEBUG_STREAM("Waypoint num: " << this->waypoint_num << " out of " << this->current_plan_msg.plan.size());
            while (loop_waypoint < current_plan_msg.plan.size() && dist_count < dist_traveled_while_planning)
            {
                if (loop_waypoint == waypoint_num) // distance between agent's current pose to first waypoint
                    dist_count += xyz_distance(this->current_plan_msg.plan[loop_waypoint].position.position, this->agent_pose.pose.position);
                else // distance between subsequent waypoints
                    dist_count += xyz_distance(this->current_plan_msg.plan[loop_waypoint].position.position, current_plan_msg.plan[loop_waypoint - 1].position.position);
                loop_waypoint++;
            }
            ROS_DEBUG_STREAM("Loop waypoint: " << loop_waypoint);
            if (loop_waypoint == current_plan_msg.plan.size())
            {
                ROS_WARN("Ran out of waypoints to plan from start at end, but there will be a pause in flight");
                // loop_waypoint--;
                return -1;
            }
            // loop_waypoint--;
            if (loop_waypoint >= current_plan_msg.plan.size())
            {
                ROS_WARN_STREAM("Loop waypoint " << loop_waypoint << " is greater than plan size " << current_plan_msg.plan.size());
                // loop_waypoint--;
                return -1;
            }
            int best_path_waypoint_index_plan_from = current_plan_waypoint_map[loop_waypoint] + 1;
            ROS_DEBUG_STREAM("Best path waypoint index plan from: " << best_path_waypoint_index_plan_from);

            // current_plan_msg.plan.erase(current_plan_msg.plan.begin()+ loop_waypoint, current_plan_msg.plan.end());
            // Trim everything past point planned from
            int erase_point = loop_waypoint;
            while (current_plan_waypoint_map[erase_point] < best_path_waypoint_index_plan_from && erase_point < current_plan_msg.plan.size())
                erase_point++;
            ROS_DEBUG_STREAM("erase_point " << erase_point);
            if (erase_point > 1)
            {
                current_plan_msg.plan.erase(current_plan_msg.plan.begin() + erase_point, current_plan_msg.plan.end());
                current_plan_waypoint_map.erase(current_plan_waypoint_map.begin() + erase_point, current_plan_waypoint_map.end());
            }
            else
            {
                current_plan_msg.plan.erase(current_plan_msg.plan.begin(), current_plan_msg.plan.end());
                current_plan_waypoint_map.erase(current_plan_waypoint_map.begin(), current_plan_waypoint_map.end());
            }
            ROS_INFO_STREAM("Size of plan after pruning " << current_plan_msg.plan.size() << " and size of waypoint map " << current_plan_waypoint_map.size());
            // set the stamp.seq of all to 0
            for (auto &waypoint_id : current_plan_waypoint_map)
                waypoint_id = 0;
            return best_path_waypoint_index_plan_from;
        }

        void trim_old_path_at_front(double time_elapsed)
        {                                          // Distance we can fly in the amount of time given to make a new plan
            time_elapsed += 2.5;                   // MAGIC NUMBER!!! TODO
            if (current_plan_msg.plan.size() == 0) //|| this->save_waypoint_index_plan_from == -1
                return;
            double dist_traveled_while_planning = this->ipp_planner->desired_speed * time_elapsed;
            double dist_count = 0;
            int loop_waypoint = this->waypoint_num_before_replan;
            ROS_DEBUG_STREAM("Current plan size: " << this->current_plan_msg.plan.size());
            while (loop_waypoint < current_plan_msg.plan.size() && dist_count < dist_traveled_while_planning)
            {
                ROS_DEBUG_STREAM("Loop Waypoint: " << loop_waypoint << " and waypoint num: " << this->waypoint_num_before_replan);
                if (loop_waypoint == waypoint_num_before_replan) // distance between agent's current pose to first waypoint
                    dist_count += xyz_distance(this->current_plan_msg.plan[loop_waypoint].position.position, this->agent_pose.pose.position);
                else // distance between subsequent waypoints
                    dist_count += xyz_distance(this->current_plan_msg.plan[loop_waypoint].position.position, current_plan_msg.plan[loop_waypoint - 1].position.position);
                loop_waypoint++;
            }
            if (loop_waypoint >= current_plan_msg.plan.size())
            {
                ROS_WARN("Out of waypoints by time this is finished. Erase all");
                current_plan_msg.plan.clear();
                current_plan_waypoint_map.clear();
                return;
            }
            ROS_INFO("Found waypoint to plan from. Getting node from best path waypoint list");
            ROS_DEBUG_STREAM("Current plan size: " << this->current_plan_msg.plan.size());
            ROS_DEBUG_STREAM("Loop waypoint: " << loop_waypoint << " is at best path waypoint " << current_plan_waypoint_map[loop_waypoint]);
            // int best_path_waypoint_index_plan_from = current_plan_msg.plan[loop_waypoint].stamp.seq;

            // keep waypoints from loop_waypoint to the next waypoint in the plan
            ROS_DEBUG_STREAM("Size of plan before pruning " << current_plan_msg.plan.size());
            ROS_DEBUG_STREAM("loop_waypoint " << loop_waypoint);
            current_plan_msg.plan.erase(current_plan_msg.plan.begin(), current_plan_msg.plan.begin() + loop_waypoint);
            current_plan_waypoint_map.erase(current_plan_waypoint_map.begin(), current_plan_waypoint_map.begin() + loop_waypoint);
            if (current_plan_waypoint_map.size() != current_plan_msg.plan.size())
                ROS_ERROR("Waypoint map and plan are different sizes");
        }

        void updated_trim_old_path_at_front()
        {
            int chop_offset = 1; //TODO: Just temporary fix for primtree, need to figure out turn-around issue
            if (this->waypoint_num + chop_offset >= current_plan_msg.plan.size())
            {
                ROS_WARN("Out of waypoints by time this is finished. Erase all");
                current_plan_msg.plan.clear();
                current_plan_waypoint_map.clear();
                return;
            }
            ROS_INFO("Updated waypoint to plan from. Getting node from best path waypoint list");
            ROS_DEBUG_STREAM("Current plan size: " << this->current_plan_msg.plan.size());
            ROS_DEBUG_STREAM("Updated: " << this->waypoint_num << " is at best path waypoint " << current_plan_waypoint_map[this->waypoint_num]);
            // int best_path_waypoint_index_plan_from = current_plan_msg.plan[loop_waypoint].stamp.seq;

            // keep waypoints from loop_waypoint to the next waypoint in the plan
            ROS_DEBUG_STREAM("Size of plan before pruning " << current_plan_msg.plan.size());
            current_plan_msg.plan.erase(current_plan_msg.plan.begin(), current_plan_msg.plan.begin() + this->waypoint_num + chop_offset);
            current_plan_waypoint_map.erase(current_plan_waypoint_map.begin(), current_plan_waypoint_map.begin() + this->waypoint_num + chop_offset);
            if (current_plan_waypoint_map.size() != current_plan_msg.plan.size())
                ROS_ERROR("Waypoint map and plan are different sizes");
        }

        /**
         * @brief Calculates the remaining available budget by subtracting the budget used
         * so far in the current plan from the total budget allowed in the Planning Request.
         * remaining budget = total_budget - budget_used
         *
         */
        double calculate_planning_budget_and_prop_info_map()
        {
            double beginning_seg_length = 0;

            // ROS_WARN_STREAM("Size of current plan " << current_plan_msg.plan.size());
            for (int i = 1; i < this->current_plan_msg.plan.size(); ++i)
            {
                double dist = xyz_distance(this->current_plan_msg.plan[i - 1].position.position, this->current_plan_msg.plan[i].position.position);
                beginning_seg_length += dist;
            }

            ROS_INFO_STREAM("The budget used by the time at start point " << beginning_seg_length);

            ROS_INFO("Planning for %f seconds", this->latest_plan_req_msg.max_planning_time);

            // Propagate info map forward to the time of the pose_to_plan_from or planning time depending on if first run or not
            double time_to_propagate;
            if (this->current_plan_msg.plan.size() == 0)
            {
                // Take distance to start point and divide by desired speed to get time
                time_to_propagate = beginning_seg_length / this->latest_plan_req_msg.desired_speed;

                // For the case where we are sitting on the start point, we need at least the planning time
                time_to_propagate = std::max(this->ipp_planner->max_planning_time, time_to_propagate);
            }
            else
            {
                // Take distance to start point given the path we already have divided by speed to get time
                time_to_propagate = beginning_seg_length / this->latest_plan_req_msg.desired_speed;
            }

            propagate_info_map(time_to_propagate); // no-op for search-only mode

            ROS_WARN_STREAM("Propagating info map for " << time_to_propagate << " seconds");
            // this->info_map->propagate_to_time(pose_to_plan_from[0]);

            double estimated_remaining_budget = remaining_budget - beginning_seg_length;

            // check if we want to return horizon instead of remaining budget
            if(this->use_plan_horizon)
            {
                if(this->horizon_length < estimated_remaining_budget)
                {
                    double horizon_remaining_budget = this->horizon_length - beginning_seg_length;
                    ROS_WARN_STREAM("Using plan budget of " << horizon_remaining_budget << " with estimated remaining budget of " << estimated_remaining_budget);
                    return horizon_remaining_budget;
                }
            }
            return estimated_remaining_budget;
        }

        void propagate_info_map(double time_to_propagate)
        {
            (void)time_to_propagate;
        }

        /**
         * @brief Check if the duration passed has met the replan time.
         *
         */
        bool is_time_to_replan()
        {
            if (!this->first_plan_request_received)
                return false;
            if (this->duration_between_replan <= ros::Duration(0))
                return false;
            return (ros::Time::now() - this->time_of_last_replan) >= this->duration_between_replan;
        }

        void visualize(std::vector<double> pose_to_plan_from, std::vector<int> waypoint_map)
        {
            if (this->visualizer)
                // this->visualizer->visualize(*this->ipp_planner);
                this->visualizer->vis_final(*this->ipp_planner, *this->info_map, pose_to_plan_from, this->current_plan_msg, waypoint_map);
            else
                ROS_WARN("No visualizer set");
        }

        /**
         * @brief Turns all parts of the trochoid path as waypoints
         *
         * @return planner_map_interfaces::Plan
         */
        void append_path_msg()
        {
            this->current_plan_msg.header.frame_id = local_frame;
            this->current_plan_msg.header.stamp = ros::Time::now();

            std::vector<og::PathGeometric> best_path_segments = this->ipp_planner->get_best_path_segments();
            tf2::Quaternion myQuaternion;
            for (int waypoint_idx = 0; waypoint_idx < best_path_segments.size(); ++waypoint_idx)
            {
                auto &path_segment = best_path_segments.at(waypoint_idx);
                for (int i = 0; i < path_segment.getStateCount()-1; ++i) // Skip the last state so there aren't double
                {
                    auto *state = path_segment.getState(i);
                    auto xyzpsi_ptr = state->as<XYZPsiStateSpace::StateType>();
                    planner_map_interfaces::Waypoint waypoint;
                    waypoint.stamp.frame_id = local_frame;
                    // waypoint.stamp.seq = waypoint_idx + 1;

                    waypoint.position.position.x = xyzpsi_ptr->getX();
                    waypoint.position.position.y = xyzpsi_ptr->getY();
                    waypoint.position.position.z = xyzpsi_ptr->getZ();
                    waypoint.waypoint_type = planner_map_interfaces::Waypoint::WAYPOINT_FLYBY;
                    myQuaternion.setRPY(0, 0, xyzpsi_ptr->getPsi());
                    waypoint.position.orientation.x = myQuaternion[0];
                    waypoint.position.orientation.y = myQuaternion[1];
                    waypoint.position.orientation.z = myQuaternion[2];
                    waypoint.position.orientation.w = myQuaternion[3];

                    this->current_plan_waypoint_map.push_back(waypoint_idx + 1);
                    this->current_plan_msg.plan.push_back(waypoint);
                }
            }
        }
    };
}

#endif
