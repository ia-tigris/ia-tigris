#pragma once

#include <cmath>
#include <vector>
#include "math_utils/math_utils.h"
#include <ros/ros.h>
#include "ipp_planners/Planner.h"

namespace ang = ca::math_utils::angular_math;
namespace ipp
{
    class RectangleCoverage
    {
    private:
        std::vector<double> sensor_offset;  // sensor_offset: offset of the sensor from the ground point wanting to be covered: x,y,z
                                            // world frame with x pointing forward, y pointing left, z pointing up
        double sensor_width;
        double turn_radius;
        double turn_curve_rad_step = M_PI_2/3.0;

    public:
        RectangleCoverage(std::vector<double> sensor_offset, double sensor_width, double turn_radius)
               :sensor_offset(sensor_offset), 
                sensor_width(sensor_width),
                turn_radius(turn_radius)
        {
            if (turn_radius*2 < sensor_width)
            {
                throw std::invalid_argument("Turn radius must two times greater than sensor width");
            }
        }
        ~RectangleCoverage()
        {

        };


        // Compute a lawnmower path using the sensor width as separation between sweeps
        // and the sensor offset as the offset from the center of the sensor
        //
        // corners: vector of corners x,y clockwise. Assumes the first point is the entry point
        //
        // returns: vector of points x,y,z,psi
        std::vector<std::vector<double>> rect_coverage(std::vector<std::vector<double>> corners)
        {
            std::vector<std::vector<double>> path;

            // Find the longer edge
            double first_edge_len = sqrt(pow(corners[1][0] - corners[0][0], 2) + pow(corners[1][1] - corners[0][1], 2));
            double second_edge_len = sqrt(pow(corners[2][0] - corners[1][0], 2) + pow(corners[2][1] - corners[1][1], 2));

            if(first_edge_len > second_edge_len)
            {
                // Calculate the psi for forward and backward pass
                double psi_forward = ang::WrapTo2Pi(atan2(corners[1][1] - corners[0][1], corners[1][0] - corners[0][0]));
                double psi_backward = ang::WrapTo2Pi(psi_forward + M_PI);

                double x_offset_forward = sensor_offset[0]*cos(psi_forward) - sensor_offset[1]*sin(psi_forward);
                double y_offset_forward = sensor_offset[0]*sin(psi_forward) + sensor_offset[1]*cos(psi_forward);
                double x_offset_backward = sensor_offset[0]*cos(psi_backward) - sensor_offset[1]*sin(psi_backward);
                double y_offset_backward = sensor_offset[0]*sin(psi_backward) + sensor_offset[1]*cos(psi_backward);
                
                double unit_x = (corners[2][0] - corners[1][0])/second_edge_len;
                double unit_y = (corners[2][1] - corners[1][1])/second_edge_len;
                double x1 = corners[0][0]+sensor_width*0.5*unit_x;
                double y1 = corners[0][1]+sensor_width*0.5*unit_y;
                double x2 = corners[1][0]+sensor_width*0.5*unit_x;
                double y2 = corners[1][1]+sensor_width*0.5*unit_y;
                int num_loops = (int)floor(second_edge_len/sensor_width);
                
                bool forward = true;
                for(int i = 0; i < num_loops; i++)
                {
                    if(forward)
                    {
                        
                        path.push_back({x1 + x_offset_forward, y1 + y_offset_forward, sensor_offset[2], psi_forward});
                        path.push_back({x2 + x_offset_forward, y2 + y_offset_forward, sensor_offset[2], psi_forward});
                        forward = false;
                    }
                    else
                    {
                        
                        path.push_back({x2 + x_offset_backward, y2 + y_offset_backward, sensor_offset[2], psi_backward});
                        path.push_back({x1 + x_offset_backward, y1 + y_offset_backward, sensor_offset[2], psi_backward});
                        forward = true;
                    }
                    x1 += sensor_width*unit_x;
                    y1 += sensor_width*unit_y;
                    x2 += sensor_width*unit_x;
                    y2 += sensor_width*unit_y;
                }
            }
            else
            {
                // Calculate the psi for forward and backward pass
                double psi_forward = ang::WrapTo2Pi(atan2(corners[2][1] - corners[1][1], corners[2][0] - corners[1][0]));
                double psi_backward = ang::WrapTo2Pi(psi_forward + M_PI);

                double x_offset_forward = sensor_offset[0]*cos(psi_forward) - sensor_offset[1]*sin(psi_forward);
                double y_offset_forward = sensor_offset[0]*sin(psi_forward) + sensor_offset[1]*cos(psi_forward);
                double x_offset_backward = sensor_offset[0]*cos(psi_backward) - sensor_offset[1]*sin(psi_backward);
                double y_offset_backward = sensor_offset[0]*sin(psi_backward) + sensor_offset[1]*cos(psi_backward);

                double unit_x = (corners[1][0] - corners[0][0])/first_edge_len;
                double unit_y = (corners[1][1] - corners[0][1])/first_edge_len;
                double x1 = corners[0][0]+sensor_width*0.5*unit_x;
                double y1 = corners[0][1]+sensor_width*0.5*unit_y;
                double x2 = corners[3][0]+sensor_width*0.5*unit_x;
                double y2 = corners[3][1]+sensor_width*0.5*unit_y;

                int num_loops = (int)floor(first_edge_len/sensor_width);
                bool forward = true;
                for(int i = 0; i < num_loops; i++)
                {
                    if(forward)
                    {
                        path.push_back({x1 + x_offset_forward, y1 + y_offset_forward, sensor_offset[2], psi_forward});
                        path.push_back({x2 + x_offset_forward, y2 + y_offset_forward, sensor_offset[2], psi_forward});
                        forward = false;
                    }
                    else
                    {
                        path.push_back({x2 + x_offset_backward, y2 + y_offset_backward, sensor_offset[2], psi_backward});
                        path.push_back({x1 + x_offset_backward, y1 + y_offset_backward, sensor_offset[2], psi_backward});
                        forward = true;
                    }
                    x1 += sensor_width*unit_x;
                    y1 += sensor_width*unit_y;
                    x2 += sensor_width*unit_x;
                    y2 += sensor_width*unit_y;
                }
            }
            return path;
        }

        // Get the length of the path for coverage over a rectangle
        double get_rect_coverage_length(std::vector<std::vector<double>> corners)
        {
            double path_length;
            
            double first_edge_len = sqrt(pow(corners[1][0] - corners[0][0], 2) + pow(corners[1][1] - corners[0][1], 2));
            double second_edge_len = sqrt(pow(corners[2][0] - corners[1][0], 2) + pow(corners[2][1] - corners[1][1], 2));

            if(first_edge_len > second_edge_len)
            {
                int num_loops = (int)floor(second_edge_len/sensor_width);
                path_length = num_loops * first_edge_len;
            }
            else
            {
                int num_loops = (int)floor(first_edge_len/sensor_width);
                path_length = num_loops * second_edge_len;
            }
            return path_length;
        }
    };

    class CoveragePlanner : public Planner
    {
    public:
        std::vector<double> pose_to_plan_from;
        std::vector<std::vector<double>> planned_path;
        double total_path_length;

        double min_x_bound = MAXFLOAT;
        double max_x_bound = -MAXFLOAT;
        double min_y_bound = MAXFLOAT;
        double max_y_bound = -MAXFLOAT;

        std::string planner_name = "coverage";

        CoveragePlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
            : Planner(nh, pnh)
        {}

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

            ROS_INFO_STREAM("[" << planner_name << "] Setup ");

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
            double declination = this->sensor_params.pitch + this->viewpoint_goal * this->sensor_params.get_vfov() / 2; // Add because pitch is from forward direction.
            double x_delta_dist = this->flight_height / tan(declination);
            
            // Compute the sensor width. No overlap in this case
            double distance = sqrt(pow(x_delta_dist, 2) + pow(this->flight_height, 2));
            double sensor_width = tan(this->sensor_params.get_hfov() / 2) * distance * 2;
            ROS_INFO_STREAM("[" << planner_name << "] sensor_width: " << sensor_width);

            RectangleCoverage rectangle_coverage({-x_delta_dist, 0, this->flight_height}, sensor_width, 1/this->max_kappa);

            std::vector<std::vector<double>> corners = {{min_x_bound, min_y_bound}, {max_x_bound, min_y_bound}, {max_x_bound, max_y_bound}, {min_x_bound, max_y_bound}};
            std::vector<std::vector<double>> rectangle_path = rectangle_coverage.rect_coverage(corners);
            for (auto point : rectangle_path)
            {
                planned_path.push_back(point);
            }
            return false;
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
    };
} // namespace ipp

