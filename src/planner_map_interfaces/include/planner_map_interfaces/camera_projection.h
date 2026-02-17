#ifndef CAMERA_PROJECTION_H
#define CAMERA_PROJECTION_H

#include <cmath>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// #include <iostream>
#include <fstream>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "planner_map_interfaces/ros_utils.h"

/**
 * Simplified and more interpretable parameters for a falloff sensor model.
 *
 * Plot this in Desmos:
 *  FNR: (0.5 - h) * (\frac{x}{m})^n + h
 *  TPR: 1 - FNR
 */
class SensorParams
{
public:
    // camera
    double focal_length;
    double width;
    double height;
    double pitch;
    // observation
    std::vector<double> max_range; // m (range between (0, inf)). means [m]ax range
    double highest_max_range;
    int model_count;
    std::vector<std::vector<double>> tpr_lookup;
    std::vector<std::vector<double>> tnr_lookup;
    std::vector<double> lookup_step_size; // assumes a constant step size
       

    SensorParams() {} // empty constructor
    SensorParams(double focal_length, double width, double height, double pitch, std::vector<double> max_range, int model_count);

    double get_vfov()
    {
        return 2 * atan(height / (2 * focal_length));
    }
    double get_hfov()
    {
        return 2 * atan(width / (2 * focal_length));
    }

    // observation model

    double fnr(double sensed_distance, int sensor_model_id) const
    {
        return 1.0 - tpr(sensed_distance, sensor_model_id);
    }

    double tpr(double sensed_distance, int sensor_model_id) const
    {
        int nearest_range_idx = std::round(sensed_distance / lookup_step_size[sensor_model_id]);
        if (nearest_range_idx >= tpr_lookup[sensor_model_id].size())
            return tpr_lookup[sensor_model_id][tpr_lookup[sensor_model_id].size()-1];
        else
            return tpr_lookup[sensor_model_id][nearest_range_idx];
    }

    double fpr(double sensed_distance, int sensor_model_id) const
    {
        return 1.0 - tnr(sensed_distance, sensor_model_id);
    }

    double tnr(double sensed_distance, int sensor_model_id) const
    {
        int nearest_range_idx = std::round(sensed_distance / lookup_step_size[sensor_model_id]);
        if (nearest_range_idx >= tnr_lookup[sensor_model_id].size())
            return tnr_lookup[sensor_model_id][tnr_lookup[sensor_model_id].size()-1];
        else
            return tnr_lookup[sensor_model_id][nearest_range_idx];
    }

    friend std::ostream& operator<<(std::ostream& out, const SensorParams& s)
	{
		out << "Focal length:" << s.focal_length << ", ";
		out << "Width: " << s.width << ", ";
		out << "Height: " << s.height << ", ";
		out << "Pitch: " << s.pitch << ", ";
		out << "Range: " << s.highest_max_range << ", " << std::endl;
		return out;
	}

};

/*This function returns the rotation of a matrix q about angles 'roll', 'pitch', 'yaw'*/
std::vector<double> rotated_q(std::vector<double> q, double roll, double pitch, double yaw);

/*This function returns the rotated camera FOV*/
std::vector<std::vector<double>> rotated_camera_fov(SensorParams sensor_params, double roll, double pitch, double yaw);
std::vector<std::vector<double>> rotated_camera_fov_w_campose(SensorParams sensor_params, geometry_msgs::Pose* cam_pose);

/*This function returns the required camera projection on the plane*/
std::vector<std::vector<double>> project_camera_bounds_to_plane(std::vector<double> agent_pos,
                                                                std::vector<std::vector<double>> q_rotated,
                                                                double sensor_cutoff_distance);

std::vector<std::vector<double>> project_raw_frustum(std::vector<double> agent_pos,
                                                     std::vector<std::vector<double>> q_rotated,
                                                     double sensor_cutoff_distance);

/**
 * @brief Given a drone pose above the zero plane, finds the projected camera bounds on that plane
 *
 * @param drone_pose
 * @param sensor_params
 * @return std::vector<std::vector<double>>
 */
std::vector<std::vector<double>> drone_pose_to_projected_camera_bounds(
    const geometry_msgs::Pose &drone_pose,
    const SensorParams &sensor_params,
    int sensor_model_id = 0);

/**
 * @brief Given a drone pose above the zero plane, finds the projected camera bounds on that plane
 *
 * @param drone_pose
 * @param sensor_params
 * @return std::vector<std::vector<double>>
 */
std::vector<std::vector<double>> drone_pose_to_projected_camera_bounds(
    const nav_msgs::Odometry &drone_pose,
    const SensorParams &sensor_params,
    int sensor_model_id = 0);

SensorParams fetch_sensor_params_from_rosparam_server(ros::NodeHandle &nh);
SensorParams fetch_sensor_params_from_rosparam_server_for_gimbal_planner(ros::NodeHandle &nh);
#endif // CAMERA_PROJECTION_H