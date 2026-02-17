#pragma once

#include <cmath>
#include <ros/ros.h>

#include "planner_map_interfaces/ros_utils.h"
#include "ipp_planners/Planner.h"
#include "ipp_planners/GreedyTrackPlanner.h"
#include "ipp_planners/InfoMapTrack.h"
#include "ipp_belief/belief_manager.h"
#include "ipp_belief/information.h"
#include "ipp_belief/trackers.h"
#include "ipp_belief/state.h"

namespace ipp
{
    class RandomTrackPlanner : public GreedyTrackPlanner
    {
    public:
        RandomTrackPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
                : GreedyTrackPlanner(nh, pnh)
        {}

        bool replan_loop(InfoMap &info_map_, std::vector<double> start_pose, double budget, bool force_from_scratch) override
        {
            auto info_map = dynamic_cast<InfoMapTrack &>(info_map_);
            // Each time we loop, we will select the next best target to visit starting from the previous choice

            // loop over all targets, calculate intersect point, and calculate the value scaled by distance to target
            int best_target_idx = -1;
            double best_value = -1;
            std::pair<double, double> best_intersect_point;

            int rand_idx = rand() % current_target_beliefs.size();
            best_target_idx = rand_idx;
            auto &best_target = current_target_beliefs[best_target_idx];
            std::pair<double, double> intersection_point = solve_soonest_intersection_drone_to_target(this->pose_to_plan_from[0], 
                                                                                                      this->pose_to_plan_from[1], 
                                                                                                      this->desired_speed, 
                                                                                                      best_target);
            best_value = calculate_value(intersection_point, best_target);
            best_intersect_point = intersection_point;

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
                    for (auto& it: this->current_target_beliefs)
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
                for (auto& it: this->planned_path)
                {
                    ROS_DEBUG_STREAM("Random Track: Planned path: " << it[0] << " " << it[1] << " " << it[2] << " " << it[3]);
                }
                ROS_INFO_STREAM("Random Track: Planned path length: " << this->total_path_length);
                return false;
            }
            else
            {
                // more targets to visit, return true
                return true;
            }
        }
    };

}