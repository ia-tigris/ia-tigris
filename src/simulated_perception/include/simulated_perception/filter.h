#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include "planner_map_interfaces/FilteredTarget.h"
#include "planner_map_interfaces/FilteredTargets.h"

class TargetFilter
{
public:
    TargetFilter();
    TargetFilter(const u_int8_t index,
                 const double observed_time,
                 const double heading,
                 const Eigen::Vector2d& first_observed_state,
                 const Eigen::Matrix2d& first_observed_cov);
    ~TargetFilter();
    void get_observation(const Eigen::Vector2d& observed_state,
                         const Eigen::Matrix2d& observed_cov,
                         const double curr_heading,
                         const double curr_observed_time);
    Eigen::Matrix<double, 5, 5> get_A_matrix();
    bool filtering();
    u_int8_t get_index();
    double get_heading();
    double get_time();
    planner_map_interfaces::FilteredTarget get_filtered_msg();
    nav_msgs::Odometry get_odom();
    void make_to_reset();
    bool ask_if_reset();

private:
    // math element
    Eigen::Matrix<double, 5, 1> prev_state; // x, y, theta(heading), speed, angular_vel
    Eigen::Matrix<double, 5, 5> prev_cov;

    Eigen::Vector3d observed_state; // x, y, theta(heading)
    Eigen::Matrix3d observed_cov;

    Eigen::Matrix<double, 5, 1> update_state;
    Eigen::Matrix<double, 5, 5> update_cov;

    Eigen::Matrix<double, 3, 5> C_mat;
    // transition uncertainty
    Eigen::Matrix<double, 5, 5> Qt = 0.0001 * Eigen::MatrixXd::Identity(5,5);

    ros::Publisher odom_vis_publisher;

    // target info
    u_int8_t target_index_;
    double last_observed_time_;
    double curr_observed_time_;
    double dt_;
    double heading_;
    bool new_observation_get_;
    bool to_reset_;
};