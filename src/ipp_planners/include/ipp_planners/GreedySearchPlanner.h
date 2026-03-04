#pragma once

#include <cmath>
#include <vector>
#include "math_utils/math_utils.h"
#include <ros/ros.h>

#include "ipp_planners/Planner.h"
#include "planner_map_interfaces/camera_projection.h"

namespace ipp
{
    class GreedySearchPlanner : public Planner
    {
    protected:
        double range;
        double min_altitude;
        double declination;

    public:
        TreeNode *parent_node;
        SearchMap local_map;
        std::vector<double> pose_to_plan_from;
        std::vector<std::vector<double>> search_bounds;
        std::vector<std::vector<double>> planned_path;
        std::vector<std::vector<double>> sampled_points;
        std::vector<std::vector<double>> view_points;
        std::vector<og::PathGeometric> planned_path_segments;

        double total_path_length;

        std::string planner_name = "greedy search";

        GreedySearchPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
            : Planner(nh, pnh)
        {
        }

        /**
         * @brief
         *
         * @param info_map the current info map
         * @param start_pose
         * @param budget
         * @param force_from_scratch
         * @return true
         * @return false
         */
        bool replan_setup(InfoMap &info_map_, std::vector<double> start_pose, double budget, bool force_from_scratch) override
        {
            // Get current belief of all targets and save the positions and velocities
            this->pose_to_plan_from = start_pose;
            local_map = *info_map_.get_search_map();
            // copy search map to copy map
            local_map.copy_map = local_map.map;

            total_path_length = 0;
            planned_path_segments.clear();
            sampled_points.clear();
            view_points.clear();

            min_altitude = flight_height;
            declination = local_map.sensor_params.pitch + viewpoint_goal 
                * local_map.sensor_params.get_vfov() / 2; // Add because pitch is from forward direction.
            range = min_altitude / std::sin(declination);

            // Add start node to tree
            ob::ScopedState<XYZPsiStateSpace> start_n(XYZPsi_Space);
            Eigen::Vector3d start_v(start_pose[0], start_pose[1], start_pose[2]);
            start_n->setXYZ(start_v);
            start_n->setPsi(start_pose[3]);

            auto *node = new TreeNode(XYZPsi_Space);
            XYZPsi_Space->getStateSpace()->copyState(node->state, start_n.get());
            node->cost = ob::Cost(0);

            double edge_reward = local_map.search_information_gain(node, true, desired_speed);

            parent_node = node;

            // extract search bounds for convex bounds checking
            search_bounds = extract_vector_polygon_bounds_from_polygon_msg(info_map_.bounds);
            
            ROS_INFO_STREAM("[" << planner_name << "] Setup ");

            return true;   
        }

        bool get_greedy_view_point(std::vector<std::vector<double>> &greedy_view_point_out, og::PathGeometric &greedy_trochoid_out)
        {
            // iterate through our search map and determine coord with max greedy reward
            double max_greedy_reward = 0.0;
            std::vector<double> max_reward_node;
            std::vector<trochoids::XYZPsiState> max_reward_trochoid;
            std::vector<double> max_reward_info; //{greedy reward, x pose, y pose, psi, euclid_distance}
            for (int row = 0; row < local_map.num_rows; row++)
            {
                for (int col = 0; col < local_map.num_cols; col++)
                {       
                    // check if point within search bounds
                    if(local_map.check_cell_inside_convex_poly(row, col, search_bounds, true))
                    {
                        // calculate reward for cell
                        double new_belief = 0;
                        double search_map_reward = local_map.calc_reward_and_belief(range, local_map.copy_map.at(row).at(col), 
                                                    new_belief, local_map.sensor_model_id.at(row).at(col)) *
                                                    local_map.priority.at(row).at(col);
                        
                        // convert grid coord to world coord
                        double grid_x = row * local_map.map_resolution + local_map.x_start + local_map.map_resolution / 2;
                        double grid_y = col * local_map.map_resolution + local_map.y_start + local_map.map_resolution / 2;

                        // calculate viewing point (pointed in direction of sampled point)
                        std::vector<double> viewing_point = find_viewing_point(pose_to_plan_from, std::make_pair(grid_x, grid_y));

                        // skip point if it's same as our pose to plan from (prevent division by 0)
                        if(viewing_point[0] != pose_to_plan_from.at(0) && viewing_point[1] != pose_to_plan_from.at(1))
                        {
                            trochoids::XYZPsiState start_state = {pose_to_plan_from[0], pose_to_plan_from[1], pose_to_plan_from[2], pose_to_plan_from[3]};
                            trochoids::XYZPsiState goal_state = {viewing_point[0], viewing_point[1], viewing_point[2], viewing_point[3]};

                            std::vector<trochoids::XYZPsiState> trochoid_path;
                            bool valid = trochoids::get_trochoid_path(start_state, goal_state, 
                            trochoid_path, this->wind, this->desired_speed, this->max_kappa, final_path_discretization_distance);

                            // calculate the length of the trochoid
                            double trochoid_dist = trochoids::get_length(trochoid_path);
                            
                            // calculate greedy reward
                            double greedy_reward = search_map_reward / trochoid_dist;

                            // set calculated reward as max reward if greater than current max
                            if(greedy_reward > max_greedy_reward)
                            {
                                max_greedy_reward = greedy_reward;
                                max_reward_trochoid = trochoid_path;
                                max_reward_node = viewing_point;
                                max_reward_info = {(double) row, (double) col, grid_x, grid_y, greedy_reward, 
                                trochoid_dist, search_map_reward, new_belief};
                            }
                        }
                    }

                }
            }
            greedy_view_point_out = {max_reward_node, max_reward_info};
            convert_trochoid_to_path_geometric(max_reward_trochoid, greedy_trochoid_out);
            return true;
        }

        bool replan_loop(InfoMap &info_map_, std::vector<double> start_pose, double budget, bool force_from_scratch) override
        {
             // get greedy sample from search map
            std::vector<std::vector<double>> greedy_sample;
            og::PathGeometric greedy_trochoid(XYZPsi_Space);
            bool valid_greedy_sample = get_greedy_view_point(greedy_sample, greedy_trochoid);
            std::vector<double> greedy_view_point = greedy_sample[0];
            // record sampled point and greedy point
            view_points.push_back(greedy_view_point);
            sampled_points.push_back({greedy_sample[1][2], greedy_sample[1][3], greedy_view_point[2], greedy_view_point[3]});

            // create TreeNode for greedy sampled point
            ob::ScopedState<XYZPsiStateSpace> new_state(this->XYZPsi_Space);
            Eigen::Vector3d new_state_v(greedy_view_point[0], greedy_view_point[1], greedy_view_point[2]);
            new_state->setXYZ(new_state_v);
            new_state->setPsi(greedy_view_point[3]);

            auto *node = new TreeNode(XYZPsi_Space);
            XYZPsi_Space->getStateSpace()->copyState(node->state, new_state.get());
            node->cost = ob::Cost(total_path_length + greedy_sample[1][5]);
            node->parent = parent_node;

            // append trochoid to our path such that we don't violate our budget
            XYZPsiStateSpace::StateType *current_state;
            XYZPsiStateSpace::StateType *next_state;

            bool budget_remaining = true;
            bool is_straight_edge_start_saved = false;
            bool is_straight_edge_end_saved = false;
            for (int i = 0; i < greedy_trochoid.getStateCount() - 1; ++i)
            {
                current_state = greedy_trochoid.getState(i)->as<XYZPsiStateSpace::StateType>();
                next_state = greedy_trochoid.getState(i+1)->as<XYZPsiStateSpace::StateType>();

                //calculate euclid distance
                double euclid_dist = compute_euclidean_distance(current_state, next_state);

                // set straight edge start pose if valid
                maybe_set_edge_start_pose(node, current_state, next_state, is_straight_edge_start_saved);
                
                // set straight edge goal pose if valid
                maybe_set_edge_end_pose(node, current_state, next_state, is_straight_edge_start_saved, is_straight_edge_end_saved);

                // check if estimated path length is within one waypoint of budget
                if((this->total_path_length + euclid_dist) > (budget - final_path_discretization_distance))
                {
                    // if we get here then we want to prune states that exceed budget
                    // greedy_trochoid.keepBefore(next_state); //keep everything before next state
                    next_state = current_state;
                    budget_remaining = false;
                    break;
                }
                this->total_path_length += euclid_dist;
            }
            // set new parent node as current node
            parent_node = node;
            // add trochoid path to our path
            planned_path_segments.push_back(std::move(greedy_trochoid));
            // set next pose to plan from as end pose of the trochoid path
            pose_to_plan_from = {next_state->getX(), next_state->getY(), next_state->getZ(), next_state->getPsi()};
            // update search map based on trochoid path
            double edge_reward = local_map.search_information_gain(node, true, desired_speed);

            return budget_remaining;
        }

        std::vector<og::PathGeometric> get_best_path_segments() override
        {
            return planned_path_segments;
        }

        const std::vector<std::vector<double>> &get_sampled_points() const
        {
            return sampled_points;
        }

        const std::vector<std::vector<double>> &get_view_points() const
        {
            return view_points;
        }
    };
}
