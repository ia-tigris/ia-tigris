#include <ros/ros.h>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include "tf/tf.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include "ipp_simple_sim/Detections.h"
#include "simulated_perception/filter.h"
#include "planner_map_interfaces/ros_utils.h"
#include "gazebo_msgs/ModelStates.h"
#include "planner_map_interfaces/camera_projection.h"

double curr_x = 0;
double curr_y = 0;
double curr_z = 0;

double curr_phi = 0;
double curr_theta = 0;
double curr_psi = 0;
Eigen::Matrix3d curr_R = Eigen::Matrix3d::Identity();

double last_observed_time = 0;
double curr_observed_time = 0;
double observation_reset_gap = 5;
bool new_observe_get = false;
bool cam_pose_init = false;

SensorParams sensor_params;

std::map<u_int8_t, ros::Publisher> vis_pub_list;
planner_map_interfaces::FilteredTargets target_list;
gazebo_msgs::ModelStates target_states;
nav_msgs::Odometry ownship_state;

double curr_time = 0;

Eigen::MatrixXd Q = 0.0001 * Eigen::MatrixXd::Identity(6,6);

ros::Publisher* observation_publisher = new ros::Publisher;
ros::Publisher* filtered_target_publisher = new ros::Publisher;

std::vector<TargetFilter> target_vec;

void cam_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    cam_pose_init = true;
    // ROS_DEBUG_STREAM("receiving cam_pose");
    curr_x = msg->pose.pose.position.x;
    curr_y = msg->pose.pose.position.y;
    curr_z = msg->pose.pose.position.z;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();
    curr_R = R;
    Eigen::Vector3d RPY = R.eulerAngles(0,1,2);
    curr_phi = RPY(0);
    curr_theta = RPY(1);
    curr_psi = RPY(2);
}

/**
 * @brief Translates a vector in the camera frame to a vector in the world frame
 * 
 * @param state state of the drone in world frame
 * @param Rot rotation of the camera in world frame
 * @param r_cam 
 * @return Eigen::Vector2d 
 */
Eigen::Vector2d get_observed_pos(const Eigen::VectorXd& state, const Eigen::Matrix3d& Rot, Eigen::Vector3d& r_cam)
{
    double x = state(0);
    double y = state(1);
    double z = state(2);

    Eigen::Vector3d ray = Rot * r_cam.normalized();
    double rx = ray(0);
    double ry = ray(1);
    double rz = ray(2);

    double observed_x = x - z*rx/rz;
    double observed_y = y - z*ry/rz;

    Eigen::Vector2d pose_vec(observed_x, observed_y);
    Eigen::Vector3d cam_pos(x,y,z);
    // ROS_INFO_STREAM("camera pose is \n " << cam_pos);
    // ROS_INFO_STREAM("camera ray value is \n " << r_cam.normalized());
    // ROS_INFO_STREAM("Rot matrix is \n " << Rot);
    // ROS_INFO_STREAM("ray in global frame is \n " << ray);
    // ROS_INFO_STREAM("projected ship is \n " << pose_vec);

    return Eigen::Vector2d(observed_x, observed_y); 
}

Eigen::Matrix2d get_cov_2d(const Eigen::VectorXd& state, const Eigen::Matrix3d& Rot, const::Eigen::MatrixXd& cov, Eigen::Vector3d& r_cam)
{
    double phi = state(3);
    double theta = state(4);
    double psi = state(5);
    double s1 = sin(phi);
    double c1 = cos(phi);
    double s2 = sin(theta);
    double c2 = cos(theta);
    double s3 = sin(psi);
    double c3 = cos(psi);

    Eigen::Vector3d ray = Rot * r_cam.normalized();
    double rx = ray(0);
    double ry = ray(1);
    double rz = ray(2);

    // std::cout << "curr_state is \n" << state << std::endl;
    // std::cout << "curr_R is \n" << Rot << std::endl;
    // std::cout << "ray in cam is \n" << r_cam << std::endl;
    // std::cout << "ray in global is \n" << ray << std::endl;

    if (rz>=0) //pointing up
    {return Eigen::Matrix2d::Identity();}

    Eigen::Matrix3d dRdphi;
    Eigen::Matrix3d dRdtheta;
    Eigen::Matrix3d dRdpsi;
    dRdphi << 0, 0, 0, 
              -s1*s3+c1*s2*c3, -s1*c3-c1*s2*s3, -c1*c2,
               c1*s3+s1*s2*c3,  c1*c3-s1*s2*s3, -s1*c2;
    dRdtheta << -s2*c3, s2*s3, c2,
                s1*c2*c3, -s1*c2*s3, s1*s2,
                -c1*c2*c3, c1*c2*s3, -c1*s2;
    dRdpsi << -c2*s3, -c2*c3, 0,
              c1*c3-s1*s2*s3, -c1*s3-s1*s2*c3, 0,
              s1*c3+c1*s2*s3, -s1*s3+c1*s2*c3, 0;

    Eigen::Vector3d draydphi = dRdphi * r_cam;
    Eigen::Vector3d draydtheta = dRdtheta * r_cam;
    Eigen::Vector3d draydpsi = dRdpsi * r_cam;

    double drxdphi = draydphi(0);
    double drydphi = draydphi(1);
    double drzdphi = draydphi(2);

    double drxdtheta = draydtheta(0);
    double drydtheta = draydtheta(1);
    double drzdtheta = draydtheta(2);

    double drxdpsi = draydpsi(0);
    double drydpsi = draydpsi(1);
    double drzdpsi = draydpsi(2);

    Eigen::MatrixXd H(2,6);
    H << 1,0, -rx/rz, curr_z*rx*drzdphi/(rz*rz), -curr_z*(rz*drxdtheta-rx*drzdtheta)/(rz*rz), -curr_z*(rz*drxdpsi-rx*drzdpsi)/(rz*rz),
         0,1, -ry/rz, curr_z*ry*drzdphi/(rz*rz), -curr_z*(rz*drydtheta-ry*drzdtheta)/(rz*rz), -curr_z*(rz*drydpsi-ry*drzdpsi)/(rz*rz);

    // std::cout << "dRdphi is \n" << dRdphi << '\n';
    // std::cout << "dRdtheta is \n" << dRdtheta << '\n';
    // std::cout << "dRdpsi is \n" << dRdpsi << '\n';
    // std::cout << "draydtheta is \n" << draydtheta << '\n';
    // std::cout << "ray is \n" << ray << '\n';
    // std::cout << "H is \n" << H << '\n';

    double x = state(0);
    double y = state(1);
    double z = state(2);

    Eigen::Matrix2d target_cov = H * cov * H.transpose();

    nav_msgs::Odometry target_pose;
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = "local_enu";
    target_pose.pose.pose.position.x = x - z*rx/rz;
    target_pose.pose.pose.position.y = y - z*ry/rz;
    target_pose.pose.pose.position.z = 0;
    target_pose.pose.covariance[0] = target_cov(0,0);
    target_pose.pose.covariance[1] = target_cov(0,1);
    target_pose.pose.covariance[6] = target_cov(1,0);
    target_pose.pose.covariance[7] = target_cov(1,1);

    observation_publisher->publish(target_pose);

    return target_cov;
}

void sensor_meas_callback(const ipp_simple_sim::Detections::ConstPtr& msg)
{
    // ROS_INFO_STREAM("receiving sensor measurement");
    // r_xcam = msg->x;
    // r_ycam = msg->y;
    // r_zcam = msg->z;
    // std::cout << "rxcam " << r_xcam << std::endl;
    if (!cam_pose_init)
    {
        ROS_INFO_STREAM("waiting for cam pose");
        return;
    }
    curr_observed_time = msg->header.stamp.toSec();
    curr_time = msg->header.stamp.toSec();
    if (msg->target_camera_vectors.size() > 0)
    {
        // ROS_INFO_STREAM("getting new measurment");
        new_observe_get = true;
        for (size_t idx = 0; idx < msg->target_camera_vectors.size(); ++idx)
        {
            bool is_new_target = true;
            u_int8_t index = msg->target_ids[idx];
            double heading = msg->headings[idx];
            Eigen::Vector3d r_cam(msg->target_camera_vectors[idx].x,
                                  msg->target_camera_vectors[idx].y,
                                  msg->target_camera_vectors[idx].z);

            Eigen::VectorXd curr_state(6);
            curr_state << curr_x, curr_y, curr_z, curr_phi, curr_theta, curr_psi;
            Eigen::Vector2d observed_state = get_observed_pos(curr_state, curr_R, r_cam);
            // ROS_INFO_STREAM("observed state is \n" << observed_state);
            Eigen::Matrix2d observed_cov = get_cov_2d(curr_state, curr_R, Q, r_cam);

            for (auto it = target_vec.begin(); it != target_vec.end(); ++it)
            {
                if (it->get_index() == index)
                {
                    it->get_observation(observed_state, 
                                        observed_cov,
                                        heading, 
                                        curr_time);
                    is_new_target = false;
                }
                // target not tracked, then add to the vector
            }
            if (is_new_target)
            {
                // ROS_INFO_STREAM("create target tracker for target " << (int)index);
                target_vec.push_back(TargetFilter(index,
                                                  curr_time,
                                                  heading,
                                                  observed_state,
                                                  observed_cov));
            }
        }
    }
    return;
}

bool kalman_updates(ros::NodeHandle &pnh)
{
    if (!new_observe_get)
    {
        // ROS_INFO_STREAM("Waiting for new observation");
        for (auto it = target_vec.begin(); it != target_vec.end(); ++it)
        {
            double last_observe_gap = curr_observed_time - it->get_time();
            if (last_observe_gap > observation_reset_gap && !(it->ask_if_reset()))
            {
                ROS_INFO_STREAM("target " << (int)(it->get_index()) << " is out of sight for too long, about to reset in next observation");
                it->make_to_reset();
            }
        }
        return false;
    }
    else
    {
        new_observe_get = false;

        target_list.targets.clear();
        target_list.header.stamp = ros::Time(curr_time);

        for (auto it = target_vec.begin(); it != target_vec.end(); ++it)
        {
            if (it->filtering())
            {
                // ROS_INFO_STREAM("filtering");
                u_int8_t target_id = it->get_index();
                target_list.targets.push_back(it->get_filtered_msg());
                auto curr_publisher = vis_pub_list.find(target_id);
                if (curr_publisher != vis_pub_list.end())
                {
                    curr_publisher->second.publish(it->get_odom());
                }
                else
                {
                    // ROS_INFO_STREAM("creating new visualizer");
                    std::string topic_name = "filtered_target_" + std::to_string(target_id) + "_vis";
                    // *filtered_target_publisher = pnh.advertise<nav_msgs::Odometry>(topic_name, 10, true);
                    vis_pub_list[target_id] = pnh.advertise<nav_msgs::Odometry>(topic_name, 10, true);
                    ROS_INFO_STREAM("new visualizer " << std::to_string(target_id) << " created");
                    vis_pub_list[target_id].publish(it->get_odom());
                }
            }
            else
            {
                double last_observe_gap = curr_observed_time - it->get_time();
                if (last_observe_gap > observation_reset_gap && !(it->ask_if_reset()))
                {
                    ROS_INFO_STREAM("target " << (int)(it->get_index()) << " is out of sight for too long, about to reset in next observation");
                    it->make_to_reset();
                }
            }
        }
        // std::cout << "latest_cov is \n" << latest_cov << std::endl;
        // target_list_publisher.publish(target_list);
        return true;
    }
}

double substitute_point_in_line(std::vector<double> &pt1, std::vector<double> &pt2, double r, double c)
/*Helper function to find the position of a point relative to a line
            > 0: Query point lies on left of the line.
            = 0: Query point lies on the line.
            < 0: Query point lies on right of the line.
*/
{
    return ((c - pt1[1]) * (pt2[0] - pt1[0])) - ((r - pt1[0]) * (pt2[1] - pt1[1]));
}

bool check_point_inside_xy_poly(double &x, double &y, std::vector<std::vector<double>> &boundary)
{
    int num_same_sides_left = 0;
    int num_same_sides_right = 0;
    for (int i = 0; i < boundary.size(); ++i)
    {
        double point_in_line = substitute_point_in_line(boundary[i],
                                                        boundary[(i + 1) % boundary.size()], x, y);
        if (point_in_line >= 0)
            num_same_sides_left++;
        if (point_in_line <= 0)
            num_same_sides_right++;
    }

    if (num_same_sides_left != boundary.size() && num_same_sides_right != boundary.size())
    {
        return false;
    }

    return true;
}

planner_map_interfaces::FilteredTarget create_filtered_msg(geometry_msgs::Pose target_pose, int index)
{
    planner_map_interfaces::FilteredTarget filtered_state;
    filtered_state.header.frame_id = "local_enu";
    filtered_state.header.stamp = ros::Time::now();

    filtered_state.local_id = index;
    filtered_state.x = target_pose.position.x;
    filtered_state.y = target_pose.position.y;
    double heading = tf2::getYaw(target_pose.orientation);
    filtered_state.xdot = 0;
    filtered_state.ydot = 0; // TODO
    std::vector<double> cov_temp{1.0, 0.0, 0.0, 0.0, 
                                0.0, 1.0, 0.0, 0.0, 
                                0.0, 0.0, 0.1, 0.0, 
                                0.0, 0.0, 0.0, 1.0};
    for (int i = 0; i < cov_temp.size(); ++i)
    {
        filtered_state.covariance[i] = cov_temp[i];
    }
    return filtered_state;
}

void airsim_targets_callback(const gazebo_msgs::ModelStates &msg) 
{
    target_states = msg;
}

void airsim_ownship_state_callback(const nav_msgs::Odometry &msg)
{
    ownship_state = msg;
}

/**
 * @brief Checks all targets in AirSim to see if within view and simulates the
 *       measurement if so.
 * 
 * @return boolean  
 */
bool airsim_targets_check()
{
    target_list.targets.clear();
    target_list.header.stamp = ros::Time::now();   

    std::vector<std::vector<double>> projected_camera_bounds = drone_pose_to_projected_camera_bounds(ownship_state, sensor_params); 

    // // get current yaw
    // tf::Quaternion q(
    // ownship_state.orientation.x,
    // ownship_state.orientation.y,
    // ownship_state.orientation.z,
    // ownship_state.orientation.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);

    // // get camera footprint
    // std::vector<std::vector<double>> q_rotated = rotated_camera_fov(sensor_params, /*roll*/ 0.0, /*pitch*/ sensor_params.pitch, /*yaw*/ yaw);
    // std::vector<double> node_pose = {ownship_state.pose.position.x, ownship_state.pose.position.y, ownship_state.pose.position.z, yaw};
    // std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pose, q_rotated, sensor_params.highest_max_range);
    
    // loop over saved target locations
    for(int i = 0; i < target_states.name.size(); i++)
    {
        // check if target is within view
        if(check_point_inside_xy_poly(target_states.pose[i].position.x, target_states.pose[i].position.y, projected_camera_bounds))
        {
            double range = std::sqrt(std::pow((target_states.pose[i].position.x - ownship_state.pose.pose.position.x), 2.0) +
                                     std::pow((target_states.pose[i].position.y - ownship_state.pose.pose.position.y), 2.0) +
                                     std::pow((target_states.pose[i].position.z - ownship_state.pose.pose.position.z), 2.0));
            double tpr_val = sensor_params.tpr(range, 0); // swap out for selected sensor model
            // random draw number between 0 and 1
            double rand_val = (double)rand() / (double)RAND_MAX;
            if(rand_val < tpr_val)
            {
                // add target to list
                target_list.targets.push_back(create_filtered_msg(target_states.pose[i], i));
            }
        }
    }

    // if none in view, return false
    return (target_list.targets.size() > 0) ? true : false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hybrid_map_demo");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh; 

    sensor_params = fetch_sensor_params_from_rosparam_server(nh);
    double loop_rate = ros_utils::get_param<double>(pnh, "loop_rate");
    observation_reset_gap = ros_utils::get_param<double>(pnh, "observation_reset_gap");
    bool simple_sim_kalman_filtering = ros_utils::get_param<bool>(pnh, "simple_sim_kalman_filtering");
    bool airsim_targets_passthrough = ros_utils::get_param<bool>(pnh, "airsim_targets_passthrough");

    // Subscribers
    ros::Subscriber cam_pose_sub = pnh.subscribe("/cam_pose", 10, cam_pose_callback);
    ros::Subscriber sensor_meas_sub = pnh.subscribe("/target_pix", 10, sensor_meas_callback);
    ros::Subscriber airsim_targets_sub = pnh.subscribe(ros_utils::get_param<std::string>(pnh, "airsim_targets_topic"), 10, airsim_targets_callback);
    ros::Subscriber airsim_ownship_sub = pnh.subscribe(ros_utils::get_param<std::string>(pnh, "airsim_ownship_topic"), 10, airsim_ownship_state_callback);
    
    // Publishers
    *observation_publisher = pnh.advertise<nav_msgs::Odometry>("observation_with_cov", 1, true);
    ros::Publisher target_list_publisher = nh.advertise<planner_map_interfaces::FilteredTargets>("filtered_targets", 1, true);

    ros::Rate r(loop_rate);
    while (pnh.ok())
    {   
        if (simple_sim_kalman_filtering && kalman_updates(pnh))
        {
            target_list_publisher.publish(target_list);
        }
        if (airsim_targets_passthrough && airsim_targets_check())
        {
            target_list_publisher.publish(target_list);
        }


        r.sleep();
        ros::spinOnce();
    }
    return 0;
}