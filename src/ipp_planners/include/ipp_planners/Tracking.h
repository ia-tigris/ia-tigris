#pragma once

#include <cmath>
#include <ros/ros.h>
#include <limits>
#include <algorithm>
#include "planner_map_interfaces/ros_utils.h"
#include "ipp_planners/Planner.h"
#include "ipp_planners/InfoMapTrack.h"
#include "ipp_planners/Topological.h"
#include "ipp_belief/belief_manager.h"
#include "ipp_belief/information.h"
#include "ipp_belief/trackers.h"
#include "ipp_belief/state.h"
#include "ipp_belief/visualize.h"
#include "ipp_belief/colors.h"


namespace ipp
{
    class TrackPlan : public Planner
    {
    protected:
        TopoTree* main_tree;
    public:
        std::vector<double> pose_to_plan_from;
        std::vector<TargetCentroid> current_target_beliefs;
        std::vector<std::vector<double>> planned_path;
        double total_path_length;

        double min_x_bound = std::numeric_limits<double>::max();
        double max_x_bound = std::numeric_limits<double>::min();
        double min_y_bound = std::numeric_limits<double>::max();
        double max_y_bound = std::numeric_limits<double>::min();

        ros::Publisher coverage_boundary_vis_pub;
        ros::Publisher coverage_particles_vis_pub;

        std::string planner_name = "tracking";

        TrackPlan(ros::NodeHandle &nh, ros::NodeHandle &pnh)
            : Planner(nh, pnh)
        {
            double is_track_task = ros_utils::get_param<bool>(pnh, "track");
            double is_search_task = ros_utils::get_param<bool>(pnh, "search");
            planner_name = ros_utils::get_param<std::string>(pnh, "planner");
            coverage_boundary_vis_pub = nh.advertise<visualization_msgs::Marker>("coverage_boundary_vis", 10);
            coverage_particles_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("coverage_particle_vis", 10);

            if (is_search_task || !is_track_task)
            {
                ROS_ERROR_STREAM(planner_name << " is developed only for track task. Please launch with search:=false track:=true");
                throw std::runtime_error("TrackPlan planner only");
            }
            main_tree = new TopoTree(XYZPsi_Space);
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
            planned_path.clear();
            total_path_length = 0;
            planned_path.push_back(start_pose); // Add start pose to the planned path

            delete main_tree;
            main_tree = new TopoTree(XYZPsi_Space);
            main_tree->initialize(info_map_, start_pose, budget, desired_speed);

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
            
            main_tree->expand_tree();
            if (main_tree->has_best_path_updated())
            {
                planned_path.clear();
                std::list<TopoNode*> new_path = main_tree->get_best_path();
                for (auto& n : new_path)
                {
                    if (n->coverage_path.empty())
                    {
                        planned_path.push_back(n->get_node_start_pose());
                    }
                    else
                    {
                        for (auto& p : n->coverage_path)
                        {
                            // ROS_WARN_STREAM("Coverage path is zero here for target " << n->get_curr_target_id());
                            planned_path.push_back(p);
                        }
                    }
                }
            }
            return true;
        }

        bool replan_teardown(InfoMap &info_map, std::vector<double> start_pose, double budget, bool force_from_scratch) override
        {
            std::string local_frame = "local_enu";
            ROS_INFO_STREAM("best path is:");
            main_tree->print_path_from_node(main_tree->get_best_path_end_node());
            coverage_boundary_vis_pub.publish(visualize_coverage_boundary(main_tree->get_path_cov_bound(), local_frame));
            visualize_coverage_particles(main_tree->get_best_path());
            return true;
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
            ROS_INFO("REturned path segments");
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
            double a = (s0 * s0) * ((V0x * V0x) + (V0y * V0y)) - (s1 * s1);
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

        const std::vector<TopoNode *> get_all_nodes() const
        {
            std::vector<TopoNode *> return_nodes;
            for (auto *node : main_tree->get_nodes())
            {
                return_nodes.push_back(node);
            }
            return return_nodes;
        }

        void visualize_coverage_particles(std::list<TopoNode*> path)
        {
            double vis_num = 20.0;
            double vis_count = 0.0;
            visualization_msgs::MarkerArray ma;
            for (auto& node : path)
            {
                for (auto &[id, tracker] : node->get_info_map_before_cov()->particle_belief_manager.id_to_trackers)
                {
                    double alpha = std::clamp((vis_num - vis_count) / vis_num, 0.0, 1.0);
                    // double alpha = 0.5;
                    tracking::ParticleFilterVisualizer visualizer(get_color(id), 50, alpha);
                    std::string ns = "propagation_particles/step" + std::to_string((int)vis_count); 
                    visualizer.update(tracker, alpha, ns);
                    ma.markers.push_back(visualizer.get_marker());
                }
                vis_count = vis_count + 1.0;
                if (vis_count > vis_num)
                {
                    break;
                }
                coverage_particles_vis_pub.publish(ma);
            }
        }
        visualization_msgs::Marker visualize_coverage_boundary(std::vector<std::vector<std::vector<double>>> target_corners,
                                                      std::string local_frame)
        {
            double visualization_scale = ros_utils::get_param<double>(pnh, "visualization_scale", 1.0);
            visualization_msgs::Marker m;
            m.header.frame_id = local_frame;
            m.header.stamp = ros::Time();
            m.ns = "nodes";
            // m.frame_locked = true;
            m.id = 0;
            m.type = visualization_msgs::Marker::LINE_LIST;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = 0.0;
            m.pose.position.y = 0.0;
            m.pose.position.z = 0.0;
            m.pose.orientation.x = 0.0;
            m.pose.orientation.y = 0.0;
            m.pose.orientation.z = 0.0;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.1 * visualization_scale;
            m.color.a = 0.9;
            m.color.r = 0.1;
            m.color.g = 0.1;
            m.color.b = 1.0;

            for (auto corners : target_corners)
            {
                geometry_msgs::Point child_point;
                child_point.x = corners[3][0];
                child_point.y = corners[3][1];
                child_point.z = corners[3][2];

                geometry_msgs::Point parent_point;
                parent_point.x = corners[0][0];
                parent_point.y = corners[0][1];
                parent_point.z = corners[0][2];

                m.points.push_back(child_point);
                m.points.push_back(parent_point);

                for (size_t i = 0; i < 3; i++)
                {
                    child_point.x = corners[i][0];
                    child_point.y = corners[i][1];
                    child_point.z = corners[i][2];

                    parent_point.x = corners[i + 1][0];
                    parent_point.y = corners[i + 1][1];
                    parent_point.z = corners[i + 1][2];

                    m.points.push_back(child_point);
                    m.points.push_back(parent_point);
                }
            }

            return m;
        }
    };

}