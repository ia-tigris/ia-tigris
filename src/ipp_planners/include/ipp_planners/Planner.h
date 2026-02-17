#pragma once

#include <chrono>
#include <vector>
#include <memory>
#include <time.h>
#include <random>
#include <limits>

#include "planner_map_interfaces/ros_utils.h"
#include "planner_map_interfaces/TargetPrior.h"
#include "planner_map_interfaces/camera_projection.h"
#include "planner_map_interfaces/PlannerStatus.h"
#include <std_msgs/Bool.h>

#include "core_planning_state_space/state_spaces/xyzpsi_state_space.h"
#include "core_planning_state_space/state_space_utils/xyzpsi_state_space_utils.h"

#include "trochoids/trochoid_utils.h"
#include "math_utils/math_utils.h"

#include "ipp_planners/InfoMap.h"
#include "ipp_planners/SearchMap.h"
#include "ipp_planners/Planner.h"

#include "ipp_belief/belief_manager.h"
#include "ipp_belief/observation.h"
#include "ipp_belief/information.h"
#include "ipp_belief/trackers.h"

namespace ang = ca::math_utils::angular_math;

typedef std::chrono::high_resolution_clock Clock;
using namespace std::chrono;

#define PI 3.141592654
#define PI_HALF 1.57079632679
#define TOLERANCE 0.00001

namespace ipp
{
    struct TargetCentroid
    {
        double x;
        double y;
        double speed;
        double heading;
        double trace;
    };

    /**
     * @brief Abstract base class planner for planning with trochoid paths for fixed wings.
     *
     */
    class Planner
    {
    public:
        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        ros::Publisher planner_heartbeat_pub;

        planner_map_interfaces::PlannerStatus planner_status_msg; // status of the planner

        /* VIEWING PARAMETERS */
        SensorParams sensor_params;

        /* LOGGING PARAMETERS */
        std::ofstream log_file;
        bool log_plan_metrics = false;
        std::ofstream final_path_file;

        /* PLANNING PARAMETERS */
        // --- Set by Plan Request ---
        double max_planning_time = 10;
        double counter_detect_radius = 0.5;
        bool include_edge = false;
        double max_plan_budget = -1; // sentinel value, make sure to set
        double final_path_discretization_distance;
        double trochoid_min_wind;

        // --- Set by ROS Param ---
        // planning distances
        bool use_ratio_distance_params = false;

        // drone movement
        double desired_speed;
        double max_kappa;
        double flight_height;

        // environment parameters
        const ompl::base::SpaceInformationPtr XYZPsi_Space;

        double viewpoint_goal = 0.6; // check that it isn't within the counterdetection?
        double wind[3] = {0, 0, 0};

        bool should_vis_while_planning = false;
        bool gimbal_planner = ros_utils::get_param<bool>(pnh, "gimbal_planner", false);

        Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
            : nh(nh),pnh(pnh),
              XYZPsi_Space(GetStandardXYZPsiSpacePtr()),
            //   sensor_params(fetch_sensor_params_from_rosparam_server(nh)),
              desired_speed(-1), // gets overwritten later by desired speed in the plan request. this is a sentinel value
              max_kappa(ros_utils::get_param<double>(pnh, "max_kappa")),
              flight_height(ros_utils::get_param<double>(pnh, "flight_height")),
              viewpoint_goal(ros_utils::get_param<double>(pnh, "viewpoint_goal")),
              should_vis_while_planning(ros_utils::get_param<bool>(this->pnh, "visualization") && ros_utils::get_param<bool>(this->pnh, "vis_while_planning")),
              final_path_discretization_distance(ros_utils::get_param<double>(this->pnh, "final_path_discretization_distance")),
              log_plan_metrics(ros_utils::get_param<bool>(this->pnh, "log_plan_metrics", false)),
            //   use_ratio_distance_params(ros_utils::get_param<bool>(this->pnh, "use_ratio_distance_params", false)),
              include_edge(ros_utils::get_param<bool>(this->pnh, "use_edge_reward", false)),
              trochoid_min_wind(ros_utils::get_param<double>(this->pnh, "trochoid_min_wind", 99999))
        {
            this->planner_heartbeat_pub = nh.advertise<planner_map_interfaces::PlannerStatus>(ros_utils::get_param<std::string>(pnh, "planner_status_topic"), 10);
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
        virtual ~Planner() = default;

        /**
         * @brief calls Replan setup, replan loop until time runs out, then replan teardown.
         *
         * @param info_map
         * @param start_pose pose of the agent
         * @param budget quantity of remaining budget
         * @param force_from_scratch completely clear any existing plan, restart planning from the start pose from scratch
         * @return true
         * @return false
         */
        template <class VisualizerClass>
        bool replan(
            InfoMap &info_map,
            std::vector<double> start_pose,
            double budget,
            bool force_from_scratch,
            VisualizerClass *visualizer = nullptr)
        {
            ROS_INFO_STREAM("\n\nStarting a replan with remaining budget: " << budget << std::endl);
            planner_status_msg.planner_status = planner_map_interfaces::PlannerStatus::STARTING_TO_PLAN;
            this->planner_heartbeat_pub.publish(planner_status_msg);

            auto start_time = ompl::time::now();

            this->max_plan_budget = budget;

            bool is_setup_success = replan_setup(info_map, start_pose, budget, force_from_scratch);
            // set up file we write to if logging
            if(this->log_plan_metrics) {
                std::string log_file_name = ros_utils::get_param<std::string>(pnh, "planner_metrics_csv_file");
                ROS_INFO_STREAM("logging plan metrics to " << log_file_name);
                this->log_file.open(log_file_name, std::ios::out | std::ios::app);
                this->log_file << "time,path_info_gain,path_cost,num_waypoints,num_nodes_in_path,num_samples,tree_size" << std::endl;
            }

            if (!is_setup_success)
            {
                ROS_ERROR("Failed to setup replan");
                return false;
            }

            /* --------- HERE IS THE MAIN LOOPY LOOP ---------- */
            ROS_INFO_STREAM("Starting to plan");
            bool continue_loop = true;
            planner_status_msg.planner_status = planner_map_interfaces::PlannerStatus::PLANNING_LOOP;
            auto last_heartbeat_time = ompl::time::now();
            this->planner_heartbeat_pub.publish(planner_status_msg);
            while (duration_cast<duration<double>>(Clock::now() - start_time).count() < this->max_planning_time && continue_loop)
            {
                // only publish if enough time has passed since last publish
                if (duration_cast<duration<double>>(Clock::now() - last_heartbeat_time).count() > 0.1) // rate limited to 10 hz
                {
                    this->planner_heartbeat_pub.publish(planner_status_msg);
                    last_heartbeat_time = ompl::time::now();
                }
                // ROS_INFO_STREAM("AM I GETTING CALLED");
                continue_loop = replan_loop(info_map, start_pose, budget, force_from_scratch);
                if (this->should_vis_while_planning)
                    visualizer->vis_while_planning(*this, info_map, start_pose);
                //log plan metrics if desired
                if(this->log_plan_metrics) {

                    this->log_file << std::fixed << ros::Time::now().toSec() << ","
                                   << this->get_plan_metrics() << std::endl;
                }

            }
            ROS_INFO_STREAM("Done planning after " << duration_cast<duration<double>>(Clock::now() - start_time).count() << " seconds");
            bool scene_initialized = false;
            if (this->pnh.getParam("scene_init", scene_initialized))
            {
                if (scene_initialized)
                {
                    this->pnh.setParam("last_launch_time", ros::Time::now().toSec());
                    ROS_WARN_STREAM("Planning finished, drone launching");
                    pnh.setParam("scene_init", false);
                }
            }
            /* ------------------------------------------------ */

            bool is_teardown_success = replan_teardown(info_map, start_pose, budget, force_from_scratch);
            // close file if logging
            if(this->log_plan_metrics)
            { 
                this->log_file.close();

                std::string final_path_file_name = ros_utils::get_param<std::string>(pnh, "final_path_csv_file");
                ROS_INFO_STREAM("logging final_path to" << final_path_file_name);
                this->final_path_file.open(final_path_file_name, std::ios::out | std::ios::app);
                this->final_path_file << "x,y,z,psi" << std::endl;
                og::PathGeometric best_paths = get_best_path();
                for(std::size_t i = 0 ; i < best_paths.getStateCount() ; ++i) {
                    XYZPsiStateSpace::StateType *current_state = best_paths.getState(i)->as<XYZPsiStateSpace::StateType>();
                    this->final_path_file << current_state->getX() << "," 
                                          << current_state->getY() << "," 
                                          << current_state->getZ() << ","
                                          << current_state->getPsi() << "\n";
                }
                this->final_path_file.close();
            }
            if (!is_teardown_success)
            {
                ROS_ERROR("Failed to teardown replan");
                return false;
            }

            return true;
        }

        /* =============================
         * ---- METHODS TO OVERRIDE ----
         * ============================= */
        /**
         * @brief Set the params from plan request msg object. May be overriden.
         * Make sure to call superclass version with `Planner::save_plan_request_params(msg)`.
         *
         * @param msg the plan request message
         */
        virtual bool save_plan_request_params(const planner_map_interfaces::PlanRequest &msg)
        {
            if (msg.planner_params.use_gimbal)
                this->sensor_params = fetch_sensor_params_from_rosparam_server_for_gimbal_planner(this->nh);
            // Planner::save_plan_request_params(msg); // uncomment me when copy pasting to subclass
            this->max_planning_time = msg.max_planning_time;
            this->counter_detect_radius = msg.counter_detection_range;
            this->max_plan_budget = msg.maximum_range;
            this->set_desired_speed(msg.desired_speed);
            double vw = sqrt(pow(msg.wind_speed.x, 2) + pow(msg.wind_speed.y, 2));
            if (vw > this->trochoid_min_wind)
            {
                ROS_INFO_STREAM("Wind speed " << vw << " > threshold " 
                                << this->trochoid_min_wind 
                                << ". Using Trochoid paths");
                this->wind[0] = msg.wind_speed.x;
                this->wind[1] = msg.wind_speed.y;
                this->wind[2] = msg.wind_speed.z;
            }
            else // Set to zero if below threshold so Dubins paths are used
            {
                ROS_INFO_STREAM("Wind speed " << vw << " < threshold " 
                                << this->trochoid_min_wind 
                                << ". Using Dubins paths");
                this->wind[0] = 0;
                this->wind[1] = 0;
                this->wind[2] = 0;
            }

            bool clear_trees = false;

            if(msg.planner_params.header.stamp.sec != 0)
            {
                if (msg.planner_params.max_kappa > 0){
                    this->max_kappa = msg.planner_params.max_kappa;
                    ROS_INFO_STREAM("Max Kappa has been reset to: " << this->max_kappa << " from plan request.");
                }
                else
                {
                    ROS_INFO_STREAM("Max Kappa was NOT reset to: " << msg.planner_params.max_kappa << " from plan request because it was <= zero.");
                }

                if (msg.planner_params.final_path_discretization_distance > 0)
                {
                    this->final_path_discretization_distance = msg.planner_params.final_path_discretization_distance;
                    ROS_INFO_STREAM("Final Path Discretization Distance has been reset to: " << this->final_path_discretization_distance << " from plan request.");
                }
                else
                {
                    ROS_INFO_STREAM("Final Path Discretization Distance was NOT reset to: " << msg.planner_params.final_path_discretization_distance << " from plan request because it was <= zero");
                }

                if (msg.planner_params.flight_height > 0){
                    if (this->flight_height != msg.planner_params.flight_height)
                    {
                        clear_trees = true; // if the flight height changes, we need to clear the trees
                        ROS_WARN_STREAM("Tree will be cleared due to flight height change");
                    }
                    this->flight_height = msg.planner_params.flight_height;
                    ROS_INFO_STREAM("Flight Height has been reset to: " << this->flight_height << " from plan request.");
                }
                else
                {
                    ROS_INFO_STREAM("Flight Height was NOT reset to: " << msg.planner_params.flight_height << " from plan request because it was <= zero");
                }
            }

            return clear_trees;
        }

        // may be overriden
        virtual bool replan_setup(InfoMap &info_map, std::vector<double> start_pose, double budget, bool force_from_scratch) { return true; }

        // must be overriden
        virtual bool replan_loop(InfoMap &info_map, std::vector<double> start_pose, double budget, bool force_from_scratch) = 0;

        // may be overriden
        virtual bool replan_teardown(InfoMap &info_map, std::vector<double> start_pose, double budget, bool force_from_scratch) { return true; }

        // may be overwritten
        virtual std::string get_plan_metrics() { return ""; }

        /**
         * @brief MUST be overriden. Returns the best path as a vector of ompl::geometric::PathGeometric objects.
         * The index of the vector corresponds to the coarse waypoint number, where the waypoint is the end of the path segment.
         * Each state in the path segment is a fine setpoint on the way to the coarse waypoint.
         *
         * @return ompl::geometric::PathGeometric
         */
        virtual std::vector<og::PathGeometric> get_best_path_segments() = 0;

        /* ===================================
         * ---- SHARED BASE CLASS METHODS ----
         * =================================== */

        std::vector<double> find_viewing_point(std::vector<double> pose_to_plan_from, std::pair<double, double> intersection_point)
        {
            double x = intersection_point.first;
            double y = intersection_point.second;

            double altitude = this->flight_height;
            double psi = std::atan2(intersection_point.second - pose_to_plan_from[1], intersection_point.first - pose_to_plan_from[0]);

            // Solve for the x, y location at the altitude and angle
            double declination = this->sensor_params.pitch + this->viewpoint_goal * this->sensor_params.get_vfov() / 2; // Add because pitch is from forward direction.
            double delta_dist = altitude / tan(declination);
            x -= delta_dist * cos(psi);
            y -= delta_dist * sin(psi);

            // Set the values in the new node
            std::vector<double> return_node = {x, y, altitude, psi};

            return return_node;
        }


        virtual og::PathGeometric get_best_path(){
            std::vector<og::PathGeometric> path_segments = this->get_best_path_segments();
            if (path_segments.size() == 0){
                ROS_WARN("No path segments in best path, returning empty path");
                return og::PathGeometric(this->XYZPsi_Space);
            }
            og::PathGeometric best_path = path_segments[0];
            for (auto it = path_segments.begin() + 1; it != path_segments.end(); ++it){
                best_path.append(*it);
            }
            return best_path;
        }

        void convert_trochoid_to_path_geometric(std::vector<trochoids::XYZPsiState> &path, og::PathGeometric &path_out)
        {
            path_out.clear();
            ob::ScopedState<XYZPsiStateSpace> output(XYZPsi_Space);
            for (std::size_t i = 0; i < path.size(); ++i)
            {
                output->setX(path[i].x);
                output->setY(path[i].y);
                output->setZ(path[i].z);
                output->setPsi(path[i].psi);
                path_out.append(output.get());
            }
        }

        void debug_check_start_point_same_as_start_node(XYZPsiStateSpace::StateType *current_state,
                                                        XYZPsiStateSpace::StateType *start_node_cast)
        {
            double distance = XYZPsi_Space->distance(current_state, start_node_cast);
            if (distance > TOLERANCE) 
            {
                // Set the precision for floating-point output
                ROS_ERROR_STREAM(std::fixed << std::setprecision(5) <<
                                "The start state of the trochoid path is not the same as the start node's state by " << distance <<
                                "\nCurrent State: (" << current_state->getX() << ", " << current_state->getY() << ", " << current_state->getZ() << ", " << current_state->getPsi() << ")" <<
                                "\nStart Node State: (" << start_node_cast->getX() << ", " << start_node_cast->getY() << ", " << start_node_cast->getZ() << ", " << start_node_cast->getPsi() << ")");
            }
        }

        /**
         * @brief Returns a feasible node that satisfies collision checking, budget, and extend distance, along the trochoid path from start to goal.
         * The feasible node's edge is stored as the portion of the Trochoid path.
         *
         * @precondition: start state must be valid
         * @author Andrew and Brady
         *
         * @param start_node
         * @param goal_node
         * @param budget
         * @param extend_dist
         * @return TreeNode* nullptr if no feasible node found
         */
        TreeNode *extend_from_start_node_toward_goal_node(InfoMap &info_map, TreeNode *start_node, TreeNode *goal_node, double extend_dist = std::numeric_limits<double>::max())
        {
            // First check if the two points are the same. If so, just return a copied state.
            if (XYZPsi_Space->distance(start_node->state, goal_node->state) < TOLERANCE)
            {
                ROS_INFO("Start and goal node are the same, returning a copied state");
                TreeNode *feasible_node = start_node->empty_clone();
                feasible_node->parent = start_node;
                feasible_node->cost = ob::Cost(start_node->cost.value());
                feasible_node->incremental_cost = ob::Cost(0);
                feasible_node->depth = start_node->depth + 1;
                feasible_node->budget_remaining = start_node->budget_remaining;
                XYZPsi_Space->getStateSpace()->copyState(feasible_node->state, start_node->state);
                return feasible_node;
            }
            
            // Step 1: solve the Trochoid
            og::PathGeometric trochoid_path(XYZPsi_Space);

            // Temporary until bubble out the changes
            XYZPsiStateSpace::StateType *start_node_cast = start_node->state->as<XYZPsiStateSpace::StateType>();
            XYZPsiStateSpace::StateType *goal_node_cast = goal_node->state->as<XYZPsiStateSpace::StateType>();
            std::vector<trochoids::XYZPsiState> trochoid_path_temp;
            trochoids::XYZPsiState start_state = {start_node_cast->getX(), start_node_cast->getY(), start_node_cast->getZ(), start_node_cast->getPsi()};
            trochoids::XYZPsiState goal_state = {goal_node_cast->getX(), goal_node_cast->getY(), goal_node_cast->getZ(), goal_node_cast->getPsi()};

            bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path_temp, this->wind, this->desired_speed, this->max_kappa, final_path_discretization_distance);
            if (!valid || trochoid_path_temp.size() == 1)
            {
                if (trochoid_path_temp.size() == 1)
                    ROS_WARN("Was asked to extend to a node at the same position????");
                return nullptr;
            }
            // Convert from trochoid path to PathGeometric
            convert_trochoid_to_path_geometric(trochoid_path_temp, trochoid_path);


            // Step 2:
            //      while conditions valid
            //          iterate along the Trochoid
            //          maybe save the edge's start pose
            //          maybe save the edge's end pose
            //          if we've reached the end of budget, then set the node to the closed set
            TreeNode *feasible_node = goal_node->empty_clone();
            feasible_node->parent = start_node;

            double dist_covered = 0;
            bool is_edge_start_saved = false;
            bool is_edge_end_saved = false;

            XYZPsiStateSpace::StateType *current_state = trochoid_path.getState(0)->as<XYZPsiStateSpace::StateType>();
            XYZPsiStateSpace::StateType *next_state;

            // debug_check_start_point_same_as_start_node(current_state, start_node_cast);

            auto euclidean_distance = [](XYZPsiStateSpace::StateType * state1, XYZPsiStateSpace::StateType * state2)
            {
                Eigen::Vector3d diff = state1->getXYZ() - state2->getXYZ();
                return diff.norm();
            };
            bool did_break = false; // TODO remove later
            for (int i = 1; i < trochoid_path.getStateCount(); i++)
            {
                next_state = trochoid_path.getState(i)->as<XYZPsiStateSpace::StateType>();
                Eigen::Vector3d next_pose(next_state->getX(), next_state->getY(), next_state->getZ());
                
                double next_state_dist = euclidean_distance(current_state, next_state);
                bool next_state_in_collision = info_map.is_collision(next_state);
                if (next_state_in_collision || 
                    (dist_covered + next_state_dist >= start_node->budget_remaining) || 
                    (dist_covered + next_state_dist >= extend_dist)) 
                {
                    if (next_state_in_collision ||
                        is_distance_nearly_exceeded(dist_covered, start_node->budget_remaining, this->final_path_discretization_distance))
                    {
                        feasible_node->is_in_closed_set = true;
                    }
                   
                    break;
                }
                // Else if no constraints violated...
                maybe_set_edge_start_pose(feasible_node, current_state, next_state, is_edge_start_saved);
                maybe_set_edge_end_pose(feasible_node, current_state, next_state, is_edge_start_saved, is_edge_end_saved);
                dist_covered += next_state_dist; // euclidean distance

                current_state = next_state;
                if (i == trochoid_path.getStateCount() - 1)
                {
                    if (!feasible_node->is_in_closed_set && is_distance_nearly_exceeded(dist_covered, start_node->budget_remaining, this->final_path_discretization_distance))
                    {
                        // Check if the last point is within one discretization distance of the budget
                        feasible_node->is_in_closed_set = true;
                    }
                }
            }
            if (!feasible_node->is_in_closed_set && is_distance_nearly_exceeded(dist_covered, start_node->budget_remaining, this->final_path_discretization_distance))
            {
                ROS_ERROR("GGOOOD THING I ADDED THIS. KEEP. Move the other out of the loop. Can remove later if fixed correctly");
                feasible_node->is_in_closed_set = true;
            }
            
            // Step 3: if reached the end condition,
            //      create a node with the current pose
            //      save the portion of the trochoid as the node's edge
            feasible_node->cost = ob::Cost(start_node->cost.value() + dist_covered);
            feasible_node->incremental_cost = ob::Cost(dist_covered);
            feasible_node->depth = start_node->depth + 1;
            feasible_node->budget_remaining = start_node->budget_remaining - dist_covered;
            // copy the current state into the feasible_node. make sure this line is above the keepBefore() line, because that line frees some memory
            XYZPsi_Space->enforceBounds(current_state);
            XYZPsi_Space->getStateSpace()->copyState(feasible_node->state, current_state); 
            trochoid_path.keepBefore(current_state); // Andrew: I believe keepBefore is inclusive, from reading OMPL's code of keeping up to closest index + 1
            feasible_node->edge_trochoid = trochoid_path;
            return feasible_node;
        }

        double compute_euclidean_distance(XYZPsiStateSpace::StateType *state1, XYZPsiStateSpace::StateType *state2)
        {
            return std::sqrt(std::pow(state1->getX() - state2->getX(), 2) + std::pow(state1->getY() - state2->getY(), 2) + std::pow(state1->getZ() - state2->getZ(), 2));
        }

        /**
         * @brief Check if the distance cap is nearly exceeded. Set an epsilon so that if we're super close we won't create a nullptr
         *
         * @param dist_covered
         * @param eps
         * @return true
         * @return false
         */
        bool is_distance_nearly_exceeded(double dist_covered, double distance_cap, double eps = 100) const //TODO epsilon should not be fixed
        {
            return dist_covered >= distance_cap - eps;
        }

        /**
         * @brief If we change from traversing the turn to traversing the straight edge, then set the node's edge start pose to the pose of the current state
         *
         * @param node
         * @param current_state
         * @param next_state
         * @param is_edge_start_saved
         * @param eps
         */
        void maybe_set_edge_start_pose(
            TreeNode *node,
            XYZPsiStateSpace::StateType *current_state,
            XYZPsiStateSpace::StateType *next_state,
            bool &is_edge_start_saved,
            double eps = 0.0001)
        {
            if (!is_edge_start_saved)
            {
                // if the heading is not changed, i.e. we're going straight, then we're traversing
                // the straight edge and we save the start pose
                if (abs(ang::TurnDirection(next_state->getPsi(), current_state->getPsi())) < eps)
                {
                    node->straight_edge_start_pose = std::vector<double>{
                        current_state->getX(), current_state->getY(),
                        current_state->getZ(), current_state->getPsi()};
                    is_edge_start_saved = true;
                }
            }
        }

        /**
         * @brief If we change from traversing the straight edge to traversing a turn, then set node's the edge end pose to the pose of the current state
         *
         * @param node
         * @param current_state
         * @param next_pose
         * @param is_edge_start_saved
         * @param is_edge_end_saved
         * @param eps
         */
        void maybe_set_edge_end_pose(
            TreeNode *node,
            XYZPsiStateSpace::StateType *current_state,
            XYZPsiStateSpace::StateType *next_state,
            bool &is_edge_start_saved,
            bool &is_edge_end_saved,
            double eps = 0.0001)
        {
            if (is_edge_start_saved && !is_edge_end_saved)
            {
                // if the heading is changed, i.e. we're turning, then we've started traversing the turn and we save the end pose
                if (abs(ang::TurnDirection(current_state->getPsi(), next_state->getPsi())) > eps)
                {
                    node->straight_edge_end_pose = std::vector<double>{
                        current_state->getX(), current_state->getY(),
                        current_state->getZ(), current_state->getPsi()};
                    is_edge_end_saved = true;
                }
            }
        }

        /* ==============================
         * ---- GETTERS AND SETTERS ----
         * ============================== */
        ompl::base::SpaceInformationPtr get_space_information_ptr() const
        {
            return XYZPsi_Space;
        }

        void set_desired_speed(double desired_speed)
        {
            this->desired_speed = desired_speed;
        }
    };

}
