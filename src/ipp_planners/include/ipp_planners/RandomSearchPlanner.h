#pragma once

#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <random>

#include "math_utils/math_utils.h"

namespace ipp
{
    class RandomSearchPlanner : public Planner
    {
    protected:
        // Random number generator
        std::random_device rd;
        std::mt19937 x_gen;
        std::mt19937 y_gen;
        std::mt19937 psi_gen;
        std::uniform_int_distribution<> x_dist;
        std::uniform_int_distribution<> y_dist;
        std::uniform_real_distribution<> distribution_psi;

    public:
        SearchMap local_map;
        std::vector<double> pose_to_plan_from;
        std::vector<std::vector<double>> search_bounds;
        std::vector<std::vector<double>> sampled_points;
        std::vector<std::vector<double>> view_points;
        std::vector<og::PathGeometric> planned_path_segments;

        double total_path_length;

        double min_x_bound = MAXFLOAT;
        double max_x_bound = -MAXFLOAT;
        double min_y_bound = MAXFLOAT;
        double max_y_bound = -MAXFLOAT;

        std::string planner_name = "random search";

        RandomSearchPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
            : Planner(nh, pnh),
            x_gen(std::mt19937(rd())),
            y_gen(std::mt19937(rd())),
            psi_gen(std::mt19937(rd())),
            distribution_psi(std::uniform_real_distribution<>(-PI, PI - 0.0001)) // OMPL wants [-pi, pi), not [0, 2pi]
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

            ROS_INFO_STREAM("[" << planner_name << "] Setup ");

            // Set min and max bounds (world coords)
            for (auto point : info_map_.bounds.points)
            {
                (point.x < min_x_bound) ? min_x_bound = point.x : min_x_bound = min_x_bound;
                (point.x > max_x_bound) ? max_x_bound = point.x : max_x_bound = max_x_bound;
                (point.y < min_y_bound) ? min_y_bound = point.y : min_y_bound = min_y_bound;
                (point.y > max_y_bound) ? max_y_bound = point.y : max_y_bound = max_y_bound;
            }
            ROS_INFO_STREAM("[" << planner_name << "] bounds: " << min_x_bound << " " << max_x_bound << " " << min_y_bound << " " << max_y_bound);
            
            // random int generator setup
            x_dist = std::uniform_int_distribution<>(min_x_bound, max_x_bound);
            y_dist = std::uniform_int_distribution<>(min_y_bound, max_y_bound);

            // convert search bounds to vector of vectors (bounds checking)
            search_bounds = extract_vector_polygon_bounds_from_polygon_msg(info_map_.bounds);

            return true;   
        }

        bool get_random_view_point(InfoMap &info_map_, std::vector<double> &rand_view_point_out, og::PathGeometric &rand_trochoid_out)
        {
            double rand_x = (double) x_dist(x_gen);
            double rand_y = (double) y_dist(y_gen);

            // check if point valid
            if(local_map.check_point_inside_convex_poly(rand_x, rand_y, search_bounds)) //TODO: replace with convex poly check instead
            {
                sampled_points.push_back(std::vector<double>({rand_x, rand_y, this->flight_height, 0.0}));

                // calculate viewing point to look at sampled point
                std::vector<double> viewing_point = find_viewing_point(pose_to_plan_from, std::make_pair(rand_x, rand_y));

                view_points.push_back(viewing_point); //view point for raw sampled point

                // create nodes for goal and sampled points
                std::vector<trochoids::XYZPsiState> rand_trochoid_path;
                trochoids::XYZPsiState start_state = {pose_to_plan_from[0], pose_to_plan_from[1], pose_to_plan_from[2], pose_to_plan_from[3]};
                trochoids::XYZPsiState goal_state = {viewing_point[0], viewing_point[1], viewing_point[2], viewing_point[3]};

                // generate trochoid path between start and goal state
                bool valid = trochoids::get_trochoid_path(start_state, goal_state, 
                                rand_trochoid_path, this->wind, this->desired_speed, this->max_kappa, final_path_discretization_distance);
                
                convert_trochoid_to_path_geometric(rand_trochoid_path, rand_trochoid_out);

                return true;
            }
            return false;
        }

        bool replan_loop(InfoMap &info_map_, std::vector<double> start_pose, double budget, bool force_from_scratch) override
        {
            std::vector<double> rand_view_point;
            og::PathGeometric rand_trochoid(XYZPsi_Space);
            bool valid_rand_sample = get_random_view_point(info_map_, rand_view_point, rand_trochoid);
            if(!valid_rand_sample)
            {
                return true;
            }

            // add trochoid path such that we don't violate budget
            XYZPsiStateSpace::StateType *current_state;
            XYZPsiStateSpace::StateType *next_state;

            bool budget_remaining = true;
            for (int i = 0; i < rand_trochoid.getStateCount() - 1; ++i)
            {
                current_state = rand_trochoid.getState(i)->as<XYZPsiStateSpace::StateType>();
                next_state = rand_trochoid.getState(i+1)->as<XYZPsiStateSpace::StateType>();

                //calculate euclid distance
                double euclid_dist = compute_euclidean_distance(current_state, next_state);

                // check if estimated path length is within one waypoint of budget
                if((this->total_path_length + euclid_dist) > (budget - final_path_discretization_distance))
                {
                    // if we get here then we want to prune states that exceed budget
                    rand_trochoid.keepBefore(next_state); //keep everything before next state
                    next_state = current_state;
                    budget_remaining = false;
                    break;
                }
                this->total_path_length += euclid_dist;
            }
            pose_to_plan_from = {next_state->getX(), next_state->getY(), next_state->getZ(), next_state->getPsi()};
            planned_path_segments.push_back(std::move(rand_trochoid));
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


// enforce bounds and plan from that point?
