#pragma once

// standard includes
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

// ROS message includes
#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>

#include <planner_map_interfaces/FilteredTarget.h>
#include <planner_map_interfaces/FilteredTargets.h>
#include <planner_map_interfaces/PlanRequest.h>
#include <planner_map_interfaces/Plan.h>
#include <planner_map_interfaces/TargetPrior.h>

#include <ipp_belief/ParticleFiltersBelief.h>
#include <ipp_belief/AddTargetPriors.h>

// source code includes
#include "ipp_belief/belief_manager.h"
#include "ipp_belief/colors.h"
#include "ipp_belief/control.h"
#include "ipp_belief/observation.h"
#include "ipp_belief/state.h"
#include "ipp_belief/information.h"
#include "planner_map_interfaces/camera_projection.h"
#include "planner_map_interfaces/ros_utils.h"

using namespace tracking;

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

/**
 * @brief Tracks the current belief of target positions
 * Subscribes to Observations and Publishes Tracker states
 *
 */
class IppBeliefNode
{
public:
    /* ----- ROS THINGS ----- */
    // node handle
    ros::NodeHandle &nh;
    ros::NodeHandle &pnh;
    // frequency
    double loop_rate;
    // whether to wait for plan
    bool pause_while_planning = false;
    bool waiting_for_plan = true;
    bool use_tf = false;

    /* SUBSCRIBERS */
    ros::Subscriber plan_request_sub; // mission parameters specified in plan request
    ros::Subscriber plan_sub;         // plan published
    ros::Subscriber agent_state_sub;

    tf2_ros::Buffer tf_buffer; // so we can look up the drone pose
    tf2_ros::TransformListener tf_listener;

    // we can construct an Observation object
    // subscribe to incoming observed targets from simulated_perception (Junbin's)
    message_filters::Subscriber<planner_map_interfaces::FilteredTargets> filtered_targets_sub;
    message_filters::Cache<planner_map_interfaces::FilteredTargets> filtered_targets_cache;

    /* PUBLISHERS */
    // publish the main belief
    ros::Publisher belief_pub;
    // publish the numeric variances of each target
    ros::Publisher total_variance_pub;
    // publish the independent variances
    ros::Publisher individual_variance_pub;

    /* SERVICES */
    ros::ServiceServer service_add_priors;
    // ros::ServiceServer service_remove_priors;
    // ros::ServiceServer service_clear_priors;

    /* ----- INTERNAL STATE ----- */

    // Stuff from the ipp_belief library
    ParticleFiltersBelief belief_manager;
    double variance_cap;

    double steps_per_second;
    SensorParams sensor_params; // so we know what the camera will cover

    std::unordered_map<unsigned int, ros::Time> target_id_to_time_last_seen;
    geometry_msgs::Pose drone_pose; // option to get drone pose from subscriber instead of tf buffer
    bool is_drone_pose_received = use_tf;

public:
    explicit IppBeliefNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh(nh), pnh(pnh),
          // SUBSCRIBERS
          tf_listener(tf_buffer),
          filtered_targets_sub(nh, ros_utils::get_param<std::string>(pnh, "filtered_targets_topic"), 10),
          filtered_targets_cache(filtered_targets_sub, 50),
          // stuff to pause propagation while planning
          pause_while_planning(ros_utils::get_param<bool>(pnh, "pause_while_planning")),
          plan_request_sub(nh.subscribe(
              ros_utils::get_param<std::string>(nh, "ipp_planners_node/plan_request_topic"),
              1, &IppBeliefNode::plan_request_callback, this)),
          plan_sub(nh.subscribe(
              ros_utils::get_param<std::string>(nh, "ipp_planners_node/plan_output_topic"),
              1, &IppBeliefNode::plan_callback, this)),
          agent_state_sub(nh.subscribe(
              ros_utils::get_param<std::string>(nh, "ipp_planners_node/agent_odom_topic"),
              1, &IppBeliefNode::odom_callback, this)),
          // PUBLISHERS
          individual_variance_pub(pnh.advertise<std_msgs::Float64MultiArray>("variance_array", 0)),
          belief_pub(
              pnh.advertise<ipp_belief::ParticleFiltersBelief>(ros_utils::get_param<std::string>(pnh, "belief_output_topic"), 100)),
          total_variance_pub(pnh.advertise<std_msgs::Float32>(ros_utils::get_param<std::string>(pnh, "variance_output_topic"), 1)),
          // SERVICES
          service_add_priors(
              pnh.advertiseService("add_target_priors_srv", &IppBeliefNode::add_priors_srv_callback, this)),
          // INTERNAL STATE
          belief_manager(pnh)
    {
        pnh.getParam("loop_rate", loop_rate);
        pnh.getParam("steps_per_second", steps_per_second);
        pnh.getParam("variance_cap", variance_cap);

        std::vector<double> max_range;
        int model_count = ros_utils::get_param<double>(nh, "sensor/model_count");
        std::vector<double> temp_max_range = ros_utils::get_param<std::vector<double>>(nh, "sensor/max_range");
        int num_max_range = static_cast<int>(temp_max_range.size());
        if (model_count > num_max_range) {
            ROS_WARN("model_count > max_range.size(). Setting model_count = max_range.size().");
            model_count = num_max_range;
        }
        for (int i = 0; i < model_count; i++)
        {
            max_range.push_back(temp_max_range.at(i));
        }
        this->sensor_params = fetch_sensor_params_from_rosparam_server(nh);
    }

    void plan_request_callback(const planner_map_interfaces::PlanRequest &msg)
    {
        this->waiting_for_plan = true;
    }

    void plan_callback(const planner_map_interfaces::Plan &msg)
    {
        this->waiting_for_plan = false;
    }

    void odom_callback(const nav_msgs::Odometry &msg)
    {
        drone_pose = msg.pose.pose;
        is_drone_pose_received = true;
    }

    bool add_priors_srv_callback(ipp_belief::AddTargetPriors::Request &req, ipp_belief::AddTargetPriors::Response &res)
    {
        auto &target_priors = req.target_priors;
        add_priors(target_priors);
        return true;
    }

    bool is_null_target_prior(const planner_map_interfaces::TargetPrior &target_prior)
    {
        bool result =
            (target_prior.target.global_id == 0) && (target_prior.target.local_id == 0) && (target_prior.target.x == 0) && (target_prior.target.y == 0) && (target_prior.target.xdot == 0) && (target_prior.target.ydot == 0) && (target_prior.target.priority == 0) && (target_prior.target.class_id == 0) && (target_prior.target.class_label.empty());
        return result;
    }

    /**
     * @brief Add more priors to the current belief. If the target ID is already in the belief,
     * resets the belief to the new prior.
     *
     * @param target_priors
     */
    void add_priors(const std::vector<planner_map_interfaces::TargetPrior> &target_priors)
    {
        ROS_INFO("Adding priors to target propagation");
        for (auto &target_prior : target_priors)
        {
            if (this->is_null_target_prior(target_prior))
            {
                // ROS_WARN("Received a null prior (id must be >0), ignoring");
                continue;
            }
            auto filtered_target = target_prior.target; // copies
            if (target_prior.target.global_id == 0)
            {
                ROS_WARN_STREAM("Received a target with global id 0, assigning new id " << target_prior.target.global_id);
                filtered_target.global_id = this->belief_manager.get_global_id_for_new_target();
            }
            bool force_add = true;
            this->belief_manager.add_new_target_to_belief(filtered_target, force_add);
        }
        ROS_INFO("All priors added");
    }

    /**
     * @brief Gets the current filtered targets and drone pose and updates the belief
     *
     * @param now
     */
    void apply_current_observation_to_belief(const ros::Time &now)
    {
        if (!is_drone_pose_received) return;
        // apply observations to belief
        try
        {
            auto filtered_targets = this->fetch_filtered_targets_for_time(now);
            auto obs = this->construct_observation(now, filtered_targets);
            // ROS_DEBUG_STREAM("Received observation with " << filtered_targets.targets.size() << " targets, applying observation to belief");
            this->belief_manager.apply_observation(obs);
        }
        catch (std::exception &ex)
        {
            ROS_WARN("SKIPPING BELIEF UPDATE: %s", ex.what());
        }
    }

    planner_map_interfaces::FilteredTargets fetch_filtered_targets_for_time(const ros::Time &now)
    {
        // get filtered targets
        planner_map_interfaces::FilteredTargets filtered_targets;
        auto filtered_targets_msg = filtered_targets_cache.getElemBeforeTime(now);
        if (filtered_targets_msg != nullptr && std::abs((now - filtered_targets_msg->header.stamp).toSec()) < 1.5)
        {
            filtered_targets = *filtered_targets_msg;
        }
        else if (filtered_targets_msg != nullptr)
        {
            // ROS_DEBUG_STREAM("Time difference was " << std::abs((now - filtered_targets_msg->header.stamp).toSec()) << " which is greater than 0.1, SKIPPING BELIEF UPDATE");
        }
        return filtered_targets;
    }

    Observation construct_observation(const ros::Time &now, const planner_map_interfaces::FilteredTargets &filtered_targets)
    { 
        if (use_tf) // look up drone position
        {
            geometry_msgs::TransformStamped drone_tf;
            // drone_tf = tf_buffer.lookupTransform("world", "tilted_cam", filtered_targets.header.stamp);
            drone_tf = tf_buffer.lookupTransform("world", "uav1/base_link", now, ros::Duration(0.1));
            // convert drone tf to a pose message
            drone_pose.position.x = drone_tf.transform.translation.x;
            drone_pose.position.y = drone_tf.transform.translation.y;
            drone_pose.position.z = drone_tf.transform.translation.z;
            drone_pose.orientation = drone_tf.transform.rotation;
        } // else, assume we subscribed to it
        return this->construct_observation_from_cam_tf(drone_pose, filtered_targets);
    }

    Observation construct_observation_from_cam_tf(geometry_msgs::Pose drone_pose, const planner_map_interfaces::FilteredTargets &filtered_targets)
    {

        // geometry_msgs::Pose drone_pose;
        // drone_pose.position.x = drone_tf.transform.translation.x;
        // drone_pose.position.y = drone_tf.transform.translation.y;
        // drone_pose.position.z = drone_tf.transform.translation.z;
        // drone_pose.orientation = drone_tf.transform.rotation;

        TargetState vantage(0, drone_pose.position.x, drone_pose.position.y, drone_pose.position.z);

        auto camera_points = drone_pose_to_projected_camera_bounds(drone_pose, this->sensor_params);

        Polygon camera_bounds(camera_points);

        // Create an Observation object from the target distributions
        Observation obs(camera_bounds, vantage, sensor_params, filtered_targets);
        return obs;
    }

    void publish_things()
    {
        // Publish belief
        belief_pub.publish(belief_manager_to_ros_msg(belief_manager));

        this->publish_variances();
    }

    void publish_variances()
    {
        std_msgs::Float64MultiArray array;
        // Clear array
        array.data.clear();
        double total_var_ = 0;
        std::map<unsigned int, double> individual_variances_map = calculate_variance_per_tracker_map(belief_manager);
        for (auto &[id, variance] : individual_variances_map)
        {
            if (variance > variance_cap)
            {
                ROS_WARN_STREAM("Variance of target " << id << " is " << variance << " which is greater than the cap of " << variance_cap << ", REMOVING TARGET");
                belief_manager.remove_tracker_id(id);
            }

            total_var_ += transform_variance(variance, variance_cap);
            array.data.push_back(transform_variance(variance, variance_cap));
        }

        individual_variance_pub.publish(array);

        std_msgs::Float32 total_var;
        total_var.data = total_var_;
        total_variance_pub.publish(total_var);
    }

    void run()
    {
        // track ros time since last propagate

        ros::Time time_of_last_propagate;

        while (ros::ok())
        {
            // option for simulation to pause propagating the belief while planning
            if (this->pause_while_planning && this->waiting_for_plan)
            {
                // do not propagate
            }
            else
            {
                auto now = ros::Time::now();

                // apply observation
                this->apply_current_observation_to_belief(now);

                // Propagate by duration since last propagation
                ros::Duration time_since_last_propagate = now - time_of_last_propagate; // ROS_DEBUG_STREAM("Propagating by " << time_since_last_propagate);
                belief_manager.propagate(time_since_last_propagate.toSec());
            }
            time_of_last_propagate = ros::Time::now();

            this->publish_things();

            ros::spinOnce(); // calls callbacks
            ros::Duration(1.0 / loop_rate).sleep();
        }
    }
};
