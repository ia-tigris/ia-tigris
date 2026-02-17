/**
 * @file InfoMap.h
 * @author your name (you@domain.com)
 * @brief An "InfoMap" differs from plain belief, in that it maps belief-->rewards, and also dictates sampling from the belief
 * @version 0.1
 * @date 2022-09-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef INFORMATIVE_BELIEF_H
#define INFORMATIVE_BELIEF_H
#include <ros/ros.h>
#include <vector>
#include <random>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <planner_map_interfaces/TargetPrior.h>
#include <planner_map_interfaces/PlanRequest.h>

#include "ipp_planners/TreeNode.h"
#include "planner_map_interfaces/camera_projection.h"

#include "ipp_planners/SearchMap.h"

namespace ipp
{
    struct KeepOutZone
    {
        std::pair<double, double> center; // x, y center of zone
        double radius;
    };
    /**
     * @brief
     *
     */
    class InfoMap
    {
    protected:
        std::mt19937 gen; // random generator

    public:
        geometry_msgs::Polygon bounds; // polygonal bounds 
        std::vector<KeepOutZone> keep_out_zones;

        // sensor model
        SensorParams sensor_params;
        double observation_discretization_distance;

        bool strict_stay_in_bounds;
        double counter_detect_radius;

        double desired_speed;
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        bool gimbal_planner = ros_utils::get_param<bool>(pnh, "gimbal_planner", false);

        /**
         * @brief Construct a new Informed Sampler object.
         * Setup a ROS subscriber to update the internal belief. MUST be overriden.
         */
        InfoMap(ros::NodeHandle &nh, ros::NodeHandle &pnh) 
        : 
        nh(nh), pnh(pnh), gen(std::random_device()()), 
        // sensor_params(fetch_sensor_params_from_rosparam_server_for_gimbal_planner(nh)), 
        strict_stay_in_bounds(ros_utils::get_param<bool>(pnh, "strict_stay_in_bounds")),
        observation_discretization_distance(ros_utils::get_param<double>(pnh, "observation_discretization_distance"))
        {
            if (gimbal_planner)
            {
                ROS_INFO_STREAM("Gimbal Planner Active");
                this->sensor_params = fetch_sensor_params_from_rosparam_server_for_gimbal_planner(nh);
            }
            else
            {
                ROS_INFO_STREAM("Gimbal Planner not active");
                this->sensor_params = fetch_sensor_params_from_rosparam_server(nh);
            }
        }

        /* =============================
         * ---- METHODS TO OVERRIDE ----
         * ============================= */
        virtual bool save_plan_request_params(const planner_map_interfaces::PlanRequest &msg){
            if (msg.planner_params.use_gimbal)
                this->sensor_params = fetch_sensor_params_from_rosparam_server_for_gimbal_planner(this->nh);
            this->desired_speed = msg.desired_speed;
            this->set_bounds(msg.search_bounds);
            this->counter_detect_radius = msg.counter_detection_range;
            
            return true;
        }

        /**
         * @brief Update the real time belief according to the latest parameters required by the planning request.
         *
         * @param msg
         * @return true if successfully sent, false if failed
         */
        virtual bool send_plan_request_params_to_belief(const planner_map_interfaces::PlanRequest &msg,  double flight_height = 80)=0;

        /**
         * @brief Gives the info map the opportunity fetch the latest belief for the current plan request.
         *
         * @param msg
         */
        virtual bool fetch_latest_belief(const std::vector<double> &pose_to_plan_from, const double &planning_budget) = 0;

        /**
         * @brief Sample an XY coordinate to add to the plan tree. MUST be overriden.
         *
         * @param tree_root current planning tree. passed in case sampling depends on the current plan tree
         * @return std::vector<double> size-2 vector of x,y coordinates
         */
        virtual std::pair<double, double> sample_xy(const TreeNode &tree_root) = 0; // must be overriden

        /**
         * @brief Calculate the information gain of a path. MUST be overriden.
         *
         * @param child_node the end node of the tree. to get the full path, traverse from the child node up its parents all the way to the root
         * @param is_edge_included whether to include edges in the information gain
         * @return double
         */
        virtual double calc_child_to_root_value(TreeNode &child_node, const bool is_edge_included = true) = 0; // must be overriden

        /**
         * @brief Check if we believe that the given state results in a collision, either with the map bounds or with internal obstacles.
         * 
         * @param state 
         * @return double 
         */
        virtual bool is_collision(XYZPsiStateSpace::StateType *state){
            return false;
        }

        /**
         * @brief Clone the info map. MUST be overriden.
         *
         * @return InfoMap* new information map
         */
        // virtual InfoMap* clone(); // =0; must be overriden. TODO: implement in the derived classes. for now just doing info map track cuz time

        /* ===================================
         * ---- SHARED BASE CLASS METHODS ----
         * =================================== */

        // /**
        //  * @brief Set the polygonal sampling bounds
        //  *
        //  */
        virtual void set_bounds(geometry_msgs::Polygon bounds)
        {
            this->bounds = bounds;
        }


        virtual void set_desired_speed(double speed)
        {
            this->desired_speed = speed;
        }

        /**
         * @brief Uniformly sample within the bounds
         *
         * @return std::pair<double, double> x,y coordinates
         */
        virtual std::pair<double, double> uniform_random_sample_xy()
        {
            // TODO: implement
            ROS_WARN("uniform_ranom_sample_xy() not yet implemented");
            return std::make_pair(0,0);
        }

        /**
         * @brief Returns the max edge size of the search map
         */
        virtual double get_max_edge_size() { 
            ROS_WARN("get_max_edge_size() not implemented. What is the use case?");
            return 0; 
        }

        virtual const SearchMap* get_search_map() const
        {   
            ROS_WARN("get_search_map() is not implemented. Returning nullptr.");
            return nullptr;
        }


        virtual ~InfoMap() = default;
    };
}
#endif // INFORMATIVE_BELIEF_H
