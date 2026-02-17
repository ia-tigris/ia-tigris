#pragma once

#include <cmath>
#include <ros/ros.h>
#include <limits>

#include "planner_map_interfaces/ros_utils.h"
#include "ipp_planners/Planner.h"
#include "ipp_planners/InfoMapTrack.h"
#include "ipp_belief/belief_manager.h"
#include "ipp_belief/information.h"
#include "ipp_belief/trackers.h"
#include "ipp_belief/state.h"

namespace ipp
{
    class GreedyTrackPlanner : public Planner
    {
    public:
        std::vector<double> pose_to_plan_from;
        std::vector<TargetCentroid> current_target_beliefs;
        std::vector<std::vector<double>> planned_path;
        double total_path_length;

        double min_x_bound = std::numeric_limits<double>::max();
        double max_x_bound = std::numeric_limits<double>::min();
        double min_y_bound = std::numeric_limits<double>::max();
        double max_y_bound = std::numeric_limits<double>::min();

        std::string planner_name = "greedy";

        GreedyTrackPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
            : Planner(nh, pnh)
        {
            double is_track_task = ros_utils::get_param<bool>(pnh, "track");
            double is_search_task = ros_utils::get_param<bool>(pnh, "search");
            planner_name = ros_utils::get_param<std::string>(pnh, "planner");
            if (is_search_task || !is_track_task)
            {
                ROS_ERROR_STREAM(planner_name << " is currently not yet implemented for search task, only track task. Please launch with search:=false track:=true");
                throw std::runtime_error("Greedy is not implemented for search task");
            }
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
            auto info_map = dynamic_cast<InfoMapTrack &>(info_map_);
            // Get current belief of all targets and save the positions and velocities
            this->pose_to_plan_from = start_pose;
            planned_path.clear();
            total_path_length = 0;
            planned_path.push_back(start_pose); // Add start pose to the planned path

            auto particle_belief_manager = info_map.get_particle_belief_manager();
            std::map<unsigned int, tracking::ParticleFilter> target_map = particle_belief_manager.get_id_to_trackers();

            for (auto &it : target_map)
            {
                tracking::TargetState tracker_state = it.second.get_mean_particle();
                TargetCentroid new_centroid = {
                    tracker_state.get_x(),
                    tracker_state.get_y(),
                    tracker_state.get_speed(),
                    tracker_state.get_heading(),
                    tracking::calculate_trace_variance(it.second)};
                current_target_beliefs.push_back(new_centroid);
            }

            ROS_INFO_STREAM("[" << planner_name << "] Setup " << current_target_beliefs.size() << " targets");
            // for(auto& it: current_target_beliefs) {
            //     ROS_WARN_STREAM("Greedy: Target " << it.x << " " << it.y << " " << it.speed << " " << it.heading << " " << it.trace);
            // }

            if (current_target_beliefs.size() == 0)
            {
                ROS_ERROR("No target belief available, cannot plan");
            }

            // Set min and max bounds
            for (auto point : info_map_.bounds.points)
            {
                (point.x < min_x_bound) ? min_x_bound = point.x : min_x_bound = min_x_bound;
                (point.x > max_x_bound) ? max_x_bound = point.x : max_x_bound = max_x_bound;
                (point.y < min_y_bound) ? min_y_bound = point.y : min_y_bound = min_y_bound;
                (point.y > max_y_bound) ? max_y_bound = point.y : max_y_bound = max_y_bound;
            }
            ROS_INFO_STREAM("[" << planner_name << "] bounds: " << min_x_bound << " " << max_x_bound << " " << min_y_bound << " " << max_y_bound);
            return true;
        }

        bool replan_loop(InfoMap &info_map_, std::vector<double> start_pose, double budget, bool force_from_scratch) override
        {
            auto info_map = dynamic_cast<InfoMapTrack &>(info_map_);
            // Each time we loop, we will select the next best target to visit starting from the previous choice

            // loop over all targets, calculate intersect point, and calculate the value scaled by distance to target
            int best_target_idx = -1;
            double best_value = -1;
            std::pair<double, double> best_intersect_point;

            for (int i = 0; i < current_target_beliefs.size(); i++)
            {
                auto &target = current_target_beliefs[i];
                // calculate intersect point
                std::pair<double, double> intersection_point = solve_soonest_intersection_drone_to_target(this->pose_to_plan_from[0],
                                                                                                          this->pose_to_plan_from[1],
                                                                                                          this->desired_speed,
                                                                                                          target);

                ROS_DEBUG("Intersection point In Loop: %f %f at speed %f", intersection_point.first, intersection_point.second, this->desired_speed);

                // calculate value
                double value = calculate_value(intersection_point, target);
                if (value > best_value)
                {
                    best_value = value;
                    best_target_idx = i;
                    best_intersect_point = intersection_point;
                }
            }

            // For the best choice, calculate the viewing point and append to path
            std::vector<double> viewing_point = find_viewing_point(pose_to_plan_from, best_intersect_point);

            ROS_DEBUG("Intersection point of best: %f %f", best_intersect_point.first, best_intersect_point.second);
            ROS_DEBUG("Viewing point of best: %f %f", viewing_point[0], viewing_point[1]);

            // Check if viewing_point is in bounds
            if (info_map_.strict_stay_in_bounds && (viewing_point[0] < min_x_bound || viewing_point[0] > max_x_bound || viewing_point[1] < min_y_bound || viewing_point[1] > max_y_bound))
            {
                ROS_DEBUG_STREAM("Viewing point " << viewing_point[0] << " " << viewing_point[1] << " is out of bounds, removing from list");
            }
            else
            {
                // If exceeded budget, remove from list and continue
                // Find distance between viewing point and pose_to_plan_from
                double distance = get_trochoid_dist(viewing_point, pose_to_plan_from);

                if (this->total_path_length + distance < budget)
                {
                    this->total_path_length += distance;
                    this->planned_path.push_back(viewing_point);
                    this->pose_to_plan_from = viewing_point;

                    // propagate the remaining targets forward based on the amount of time it would take to reach the viewing point
                    double time_to_reach_viewing_point = distance / this->desired_speed;
                    for (auto &it : this->current_target_beliefs)
                    {
                        it.x += time_to_reach_viewing_point * it.speed * std::cos(it.heading);
                        it.y += time_to_reach_viewing_point * it.speed * std::sin(it.heading);
                    }
                }
                else
                {
                    ROS_DEBUG_STREAM("Budget exceeded, removing target from list");
                }
            }

            // remove target from current_target_beliefs
            this->current_target_beliefs.erase(this->current_target_beliefs.begin() + best_target_idx);

            if (current_target_beliefs.size() == 0 || this->total_path_length > budget)
            {
                // no more targets to visit, return false
                for (auto &it : this->planned_path)
                {
                    ROS_DEBUG_STREAM("[Greedy] Planned path: " << it[0] << " " << it[1] << " " << it[2] << " " << it[3]);
                }
                ROS_INFO_STREAM("[Greedy] Planned path length: " << this->total_path_length);
                return false;
            }
            else
            {
                // more targets to visit, return true
                return true;
            }
        }

        std::vector<og::PathGeometric> get_best_path_segments() override
        {
            std::vector<og::PathGeometric> segments;
            ROS_WARN_STREAM("Size of " << planner_name << " path: " << this->planned_path.size());
            for (auto it = planned_path.begin(); it + 1 != planned_path.end(); ++it)
            {
                auto waypoint = *it;
                auto next_waypoint = *(it + 1);
                
                trochoids::XYZPsiState start_state = {waypoint[0], waypoint[1], waypoint[2], waypoint[3]};
                trochoids::XYZPsiState goal_state = {next_waypoint[0], next_waypoint[1], next_waypoint[2], next_waypoint[3]};
                std::vector<trochoids::XYZPsiState> trochoid_path;
                bool valid = trochoids::get_trochoid_path(
                    start_state, goal_state, trochoid_path, this->wind, this->desired_speed, this->max_kappa, final_path_discretization_distance);
                
                og::PathGeometric path_segment(this->XYZPsi_Space);
                convert_trochoid_to_path_geometric(trochoid_path, path_segment);
                segments.push_back(std::move(path_segment));
            }
            ROS_INFO("Returned path segments");
            return segments;
        }

        /**
         * @brief find the xy location of the soonest theoretic intersection that the drone could make to the particle.
         * TODO: take into account drone heading and motion model. this is a 20/80 solution
         * https://stackoverflow.com/a/22117046/5118517
         * @param drone_x
         * @param drone_y
         * @param drone_speed
         * @param target_pose
         * @return std::pair<double, double>  returns <nan, nan> if no solution. i.e. particle faster than drone
         */
        static std::pair<double, double> solve_soonest_intersection_drone_to_target(double drone_x, double drone_y, double drone_speed, TargetCentroid &target_pose)
        {
            double P0x = target_pose.x;
            double P0y = target_pose.y;
            double s0 = target_pose.speed;
            double V0x = std::cos(target_pose.heading);
            double V0y = std::sin(target_pose.heading);
            double P1x = drone_x, P1y = drone_y;
            double s1 = drone_speed;

            // quadratic formula
            double a = (s0 * s0) - (s1 * s1);
            // if (abs((V0x * V0x) + (V0y * V0y) - 1) > 0.000001)
            // {
            //     ROS_ERROR_STREAM("Squared and summed not zero?: " << ((V0x * V0x) + (V0y * V0y)));
            // }
            double b = 2 * s0 * ((P0x * V0x) + (P0y * V0y) - (P1x * V0x) - (P1y * V0y));
            double c = (P0x * P0x) + (P0y * P0y) + (P1x * P1x) + (P1y * P1y) - (2 * P1x * P0x) - (2 * P1y * P0y);

            double t = NAN;
            if (a == 0)
            {
                t = -c / b;
            }
            else
            {
                double t1 = (-b + std::sqrt((b * b) - (4 * a * c))) / (2 * a);
                double t2 = (-b - std::sqrt((b * b) - (4 * a * c))) / (2 * a);

                t = choose_best_time(t1, t2);
            }

            if (isnan(t))
            {
                ROS_WARN_STREAM(
                    "No solution for intersection: "
                    << "drone_x=" << drone_x << " drone_y=" << drone_y << " drone_speed=" << drone_speed
                    << " paricle x= " << target_pose.x << " particle y= " << target_pose.y
                    << " particle speed= " << target_pose.speed << " particle heading= " << target_pose.heading);
                return std::make_pair(t, t); // returns nans
            }

            double intersect_x = P0x + t * s0 * V0x;
            double intersect_y = P0y + t * s0 * V0y;
            return std::make_pair(intersect_x, intersect_y);
        }

        double calculate_value(std::pair<double, double> intersection_point, TargetCentroid &target) const
        {
            // Find distance from intersection point to pose_to_plan_from
            double distance = std::sqrt(std::pow(intersection_point.first - this->pose_to_plan_from[0], 2) +
                                        std::pow(intersection_point.second - this->pose_to_plan_from[1], 2));
            // Return the value scaled by the distance
            return target.trace / distance;
        }

        static double choose_best_time(double t1, double t2)
        {
            auto positive_and_not_nan = [](double t)
            { return t > 0 && !std::isnan(t); };

            if (positive_and_not_nan(t1) && positive_and_not_nan(t2))
            {
                return std::min(t1, t2);
            }
            else if (positive_and_not_nan(t1))
            {
                return t1;
            }
            else if (positive_and_not_nan(t2))
            {
                return t2;
            }
            else
            {
                return std::nan("no solution");
            }
        }

        double get_trochoid_dist(std::vector<double> &viewing_point, std::vector<double> pose_to_plan_from)
        {
            trochoids::XYZPsiState start_state = {pose_to_plan_from[0], pose_to_plan_from[1], pose_to_plan_from[2], pose_to_plan_from[3]};
            trochoids::XYZPsiState goal_state = {viewing_point[0], viewing_point[1], viewing_point[2], viewing_point[3]};

            return trochoids::get_length(start_state, goal_state, this->wind, this->desired_speed, this->max_kappa);
        }
    };

}