#pragma once

// C++ std
#include <chrono>
#include <cmath>
#include <string>
#include <time.h>

// ROS native
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

// our custom messages
#include <planner_map_interfaces/Plan.h>
#include <planner_map_interfaces/PlanRequest.h>
#include <planner_map_interfaces/Waypoint.h>
#include <ipp_simple_sim/Detections.h>

// utils
#include "planner_map_interfaces/camera_projection.h"
#include "planner_map_interfaces/ros_utils.h"

// planners
#include "ipp_planners/Planner.h"
#include "ipp_planners/Tigris.h"
#include "ipp_planners/MCTS.h"

// belief
#include "ipp_planners/InfoMap.h"
#include "ipp_planners/InfoMapTrack.h"
#include "ipp_planners/InfoMapSearchTrack.h"

#include "ipp_planners/SearchMap.h"
#include "ipp_planners/SearchMapSetup.h"
#include "ipp_planners/SearchMapUpdate.h"

// visuals
#include "ipp_planners/visualizations.h"

#include "ipp_planners/ipp_planners_node.h"
#include <unordered_map>

using namespace ros_utils;

namespace ipp
{
    class PlannerNodeMC : public PlannerNode
    {
        InfoMapTrack* info_map_track;

        ros::Publisher plan_request_pub;
        ros::Publisher kill_all_nodes_pub;

        bool init_new_planner_and_plan = true;
        std::vector<std::string> planner_types;
        int planner_type_index = -1;
        bool received_sim_startup_msg = false; 
        int loop_count = 0;
        int number_of_trials;
        planner_map_interfaces::PlanRequest plan_request_msg;
        int tester = 1;
        int boundary_offset_for_search_map_reset = 0;

        bool execute_plan = true;
        bool sample_plan_request_randomly = true;

        bool sample_targets = false;
        bool log_plan_metrics = false;
        bool log_search_map_metrics = false;
        double flight_height = 110.0;
        
        // realtime search map logging
        float average_entropy;
        float average_prob;
        ros::Subscriber average_entropy_sub;
        ros::Subscriber average_prob_sub;
        std::ofstream realtime_search_map_log_file;

        //planner metric logging (when execute_plan = true)
        std::ofstream planner_log_file;
        ros::Subscriber plan_sub;

        //plan logging (when execute_plan = true)
        std::ofstream plan_log_file;
        ros::Subscriber waypoint_sub;

        //target logging (when sample_targets = true & execute_plan = true)
        std::ofstream target_log_file;
        std::ofstream gt_target_log_file;
        std::vector<int> detected_target_ids;
        ros::Subscriber target_sub;

    public:
        /**
         * @brief Construct a new Planning Node object
         *
         * @param nh
         */
        PlannerNodeMC(ros::NodeHandle &nh, ros::NodeHandle &pnh)
            : PlannerNode(nh, pnh, false)
        {
            ROS_INFO("Use the right constructor");
            init_subscribers();
            init_publishers();
            planner_types = ros_utils::get_param<std::vector<std::string>>(pnh, "planner_types");
            execute_plan = ros_utils::get_param<bool>(pnh, "execute_plan");
            sample_plan_request_randomly = ros_utils::get_param<bool>(pnh, "sample_plan_request_randomly");
            number_of_trials = ros_utils::get_param<int>(pnh, "number_of_trials");
            log_search_map_metrics = ros_utils::get_param<bool>(pnh, "log_search_map_metrics");
            flight_height = ros_utils::get_param<double>(pnh, "flight_height");
            nh.setParam("scene_init", false);

            if(ros_utils::get_param<std::string>(pnh, "use_sim") == "false")
            {
                ROS_INFO("Not using simulator");
                this->received_sim_startup_msg = true;
            }
        }

        /**
         * @brief Main mc loop of the node.
         *
         */
        void run_mc();

        void init_attributes();
        void init_subscribers();
        void init_publishers();

        // update plan request
        void plan_request_callback(const planner_map_interfaces::PlanRequest &msg);

        // update the state of our agent
        void remaining_budget_callback(const std_msgs::Float32 &msg);

        // update the state of our agent if input is posestamp msg
        void agent_state_callback(const geometry_msgs::PoseStamped &msg);

        // update the state of our agent if input is odom msg
        void agent_odom_callback(const nav_msgs::Odometry &msg);

        // subscribes to check what waypoint index has been reached
        void waypoint_reached_callback(const std_msgs::UInt32 &msg);

        // subscribes to average entropy pub for average entropy of search map
        void average_entropy_callback(const std_msgs::Float32 &msg);

        // subscribes to path published by planner
        void plan_callback(const planner_map_interfaces::Plan &msg);

        //subscribes to the waypoint number published by simple sim
        void waypoint_num_callback(const std_msgs::UInt32 &msg);

        void average_prob_callback(const std_msgs::Float32 &msg);

        void target_detection_callback(const ipp_simple_sim::Detections &msg);

        void propagate_info_map(double time_to_propagate, 
                                std::vector<tracking::Observation> &observations,
                                std::vector<double> time_deltas);

        planner_map_interfaces::PlanRequest samplePlanRequest(std::vector<planner_map_interfaces::TargetPrior> target_priors);

        std::vector<planner_map_interfaces::TargetPrior> generateTargetPriors();

        std::vector<planner_map_interfaces::TargetPrior> generateSearchPriors();

        bool all_targets_outside_of_bounds();
    };
}
