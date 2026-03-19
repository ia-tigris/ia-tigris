#include "planner_map_interfaces/camera_projection.h"
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

SensorParams::SensorParams(double focal_length, double width, double height, double pitch, std::vector<double> max_range, int model_count)
        : focal_length(focal_length), width(width), height(height), pitch(pitch), max_range(max_range), model_count(model_count)
{
    highest_max_range = max_range[0];
    for (int i = 1; i < model_count; ++i)
    {
        if (max_range[i] > highest_max_range)
        {
            highest_max_range = max_range[i];
        }
    }

    ros::NodeHandle nh;
    std::string path = ros_utils::get_param<std::string>(nh, "sensor/sensor_params_path", "/home/junbin/ipp_ws/src/planner_map_interfaces/config/tracking");
    // initialize lookup tables
    for (int i = 0; i < model_count; ++i)
    {
        // ROS_INFO("Initializing lookup table for model %d", i);
        std::stringstream ss;
        std::string line, word;
        

        ss << path << "/sensor_model_" << i << ".csv";
        if (!fs::exists(fs::path(ss.str()))){
            throw std::runtime_error("Sensor model file not found: " + ss.str());
        }
        std::vector<double> tpr_lookup_i;
        std::vector<double> tnr_lookup_i;
        std::ifstream file(ss.str());
        bool title_line = true; 
        bool first_line = false; 
        bool second_line = false;
        double x_val = -1;
        double first_x = -1;

        while(getline(file, line))
        {
            if (title_line) //skip the title line
            {
                title_line = false;
                first_line = true;
                continue;
            }

            std::stringstream str(line);
            int index = 0;
            while(getline(str, word, ','))
            {
                if (index == 0)
                    x_val = stod(word);
                else if (index == 1)
                    tpr_lookup_i.push_back(stod(word));
                else if (index == 2)
                    tnr_lookup_i.push_back(stod(word));
                else
                    ROS_ERROR("Error in sensor model file. Should only be 3 columns.");
                ++index;
            }

            if (first_line) // save value of x to find step size
            {
                if (x_val < 0)
                    ROS_ERROR("Error in sensor model file. First value is less than 0.");
                first_line = false;
                second_line = true;
                first_x = x_val;
            }
            else if (second_line) // compute the step size
            {
                if (x_val < 0 || x_val < first_x || first_x < 0)
                    ROS_ERROR("Error in sensor model file. Second value error.");
                second_line = false;
                lookup_step_size.push_back(x_val - first_x);
            }
        }
        tpr_lookup.push_back(tpr_lookup_i);
        tnr_lookup.push_back(tnr_lookup_i);
    }
}

std::vector<double> rotated_q(std::vector<double> q, double roll, double pitch, double yaw)
{
    double x = std::cos(yaw) * std::cos(pitch) * q[0] + (std::cos(yaw) * std::sin(pitch) * std::sin(roll) - std::sin(yaw) * std::cos(roll)) * q[1] + (std::cos(yaw) * std::sin(pitch) * std::cos(roll) + std::sin(yaw) * std::sin(roll)) * q[2];
    double y = std::sin(yaw) * std::cos(pitch) * q[0] + (std::sin(yaw) * std::sin(pitch) * std::sin(roll) + std::cos(yaw) * std::cos(roll)) * q[1] + (std::sin(yaw) * std::sin(pitch) * std::cos(roll) - std::cos(yaw) * std::sin(roll)) * q[2];
    double z = -std::sin(pitch) * q[0] + std::cos(pitch) * std::sin(roll) * q[1] + std::cos(pitch) * std::cos(roll) * q[2];
    std::vector<double> q_return = {x, y, z};
    return q_return;
}

std::vector<double> rotated_q_quat(std::vector<double> q, Eigen::Quaterniond Quat)
{
    Eigen::Vector3d q_vec(q[0], q[1], q[2]);
    Eigen::Vector3d q_rotated;
    q_rotated = Quat * q_vec;
    std::vector<double> q_return = {q_rotated.x(), q_rotated.y(), q_rotated.z()};
    return q_return;
}

std::vector<std::vector<double>> rotated_camera_fov(SensorParams sensor_params, double roll, double pitch, double yaw)
{
    std::vector<double> q1 = {(double) sensor_params.focal_length, (double) -sensor_params.width / 2.0, (double) -sensor_params.height / 2.0};
    std::vector<double> q2 = {(double) sensor_params.focal_length, (double) -sensor_params.width / 2.0, (double) sensor_params.height / 2.0};
    std::vector<double> q3 = {(double) sensor_params.focal_length, (double) sensor_params.width / 2.0, (double) sensor_params.height / 2.0};
    std::vector<double> q4 = {(double) sensor_params.focal_length, (double) sensor_params.width / 2.0, (double) -sensor_params.height / 2.0};
    q1 = rotated_q(q1, roll, pitch, yaw);
    q2 = rotated_q(q2, roll, pitch, yaw);
    q3 = rotated_q(q3, roll, pitch, yaw);
    q4 = rotated_q(q4, roll, pitch, yaw);

    std::vector<std::vector<double>> q = {q1, q2, q3, q4};

    return q;
}

std::vector<std::vector<double>> rotated_camera_fov_w_campose(SensorParams sensor_params, geometry_msgs::Transform* cam_pose)
{
    std::vector<double> q1 = {(double) sensor_params.focal_length, (double) -sensor_params.width / 2.0, (double) -sensor_params.height / 2.0};
    std::vector<double> q2 = {(double) sensor_params.focal_length, (double) -sensor_params.width / 2.0, (double) sensor_params.height / 2.0};
    std::vector<double> q3 = {(double) sensor_params.focal_length, (double) sensor_params.width / 2.0, (double) sensor_params.height / 2.0};
    std::vector<double> q4 = {(double) sensor_params.focal_length, (double) sensor_params.width / 2.0, (double) -sensor_params.height / 2.0};
    Eigen::Quaterniond cam_rot(cam_pose->rotation.w, cam_pose->rotation.x, cam_pose->rotation.y, cam_pose->rotation.z);
    q1 = rotated_q_quat(q1, cam_rot);
    q2 = rotated_q_quat(q2, cam_rot);
    q3 = rotated_q_quat(q3, cam_rot);
    q4 = rotated_q_quat(q4, cam_rot);

    std::vector<std::vector<double>> q = {q1, q2, q3, q4};

    return q;
}

std::vector<std::vector<double>> project_raw_frustum(std::vector<double> agent_pos,
                                                     std::vector<std::vector<double>> q_rotated,
                                                     double sensor_cutoff_distance)
{
    std::vector<std::vector<double>> sphere_intercept;
    if (agent_pos[2] <= -0.5)
    {
        ROS_ERROR_STREAM("drone under ground?");
        // return projected_camera_bounds;
    }
    bool reach_ground = false;

    for (size_t j = 0; j < q_rotated.size(); ++j)
    {
        double length_q = sqrt(q_rotated[j][0] * q_rotated[j][0] + q_rotated[j][1] * q_rotated[j][1] + q_rotated[j][2] * q_rotated[j][2]);
        double intercept_x = agent_pos[0] + (sensor_cutoff_distance / length_q) * q_rotated[j][0];
        double intercept_y = agent_pos[1] + (sensor_cutoff_distance / length_q) * q_rotated[j][1];
        double intercept_z = agent_pos[2] + (sensor_cutoff_distance / length_q) * q_rotated[j][2];
        if (intercept_z < 0)
        {
            reach_ground = true;
        }
        std::vector<double> point_on_sphere = {intercept_x, intercept_y, intercept_z};
        sphere_intercept.push_back(point_on_sphere);
    }

    return sphere_intercept;
}

double wrap_to_mpi_pi(double rad_in)
{
    double rad_out = rad_in;
    while (rad_out < -M_PI)
    {
        rad_out += 2.0 * M_PI;
    }

    while (rad_out >= M_PI)
    {
        rad_out -= 2.0 * M_PI;
    }

    return rad_out;
}

double wrap_to_0_2pi(double rad_in)
{
    double rad_out = rad_in;
    while (rad_out < 0)
    {
        rad_out += 2.0 * M_PI;
    }

    while (rad_out >= 2.0 * M_PI)
    {
        rad_out -= 2.0 * M_PI;
    }

    return rad_out;
}

/*This function returns the required camera projection on the plane*/
std::vector<std::vector<double>> project_camera_bounds_to_plane(std::vector<double> agent_pos,
                                                                std::vector<std::vector<double>> q_rotated,
                                                                double sensor_cutoff_distance)
{
    std::vector<std::vector<double>> projected_camera_bounds;
    std::vector<std::vector<double>> sphere_intercept;
    if (agent_pos[2] <= -0.5)
    {
        ROS_ERROR_STREAM("drone under ground?");
        // return projected_camera_bounds;
    }
    bool reach_ground = false;

    for (size_t j = 0; j < q_rotated.size(); ++j)
    {
        double length_q = sqrt(q_rotated[j][0] * q_rotated[j][0] + q_rotated[j][1] * q_rotated[j][1] + q_rotated[j][2] * q_rotated[j][2]);
        double intercept_x = agent_pos[0] + (sensor_cutoff_distance / length_q) * q_rotated[j][0];
        double intercept_y = agent_pos[1] + (sensor_cutoff_distance / length_q) * q_rotated[j][1];
        double intercept_z = agent_pos[2] + (sensor_cutoff_distance / length_q) * q_rotated[j][2];
        if (intercept_z < 0)
        {
            reach_ground = true;
        }
        std::vector<double> point_on_sphere = {intercept_x, intercept_y, intercept_z};
        sphere_intercept.push_back(point_on_sphere);
    }

    if (!reach_ground)
    {
        ROS_WARN_STREAM("Camera range is so small that nothing on the sea level is considered to be detectable. Check the Z of the agent?");
        // return projected_camera_bounds;
    }
    // get the intersect with the plane
    // check the first corner ray and the edge between 1-4 if there is a ray pointing up
    std::vector<double> first_ray_end = sphere_intercept[0];
    std::vector<double> last_ray_end = sphere_intercept[3];

    // check if the two ray points generate an intersect with the first ray's frustum edge plane and the ground
    // in this case, one end points z > 0 and one end point z < 0
    if (last_ray_end[2] * first_ray_end[2] < 0) // Top edge of image frame
    {
        double theta0 = acos(-agent_pos[2] / sensor_cutoff_distance);
        double theta1 = acos((last_ray_end[2] - agent_pos[2]) / sensor_cutoff_distance);
        double theta2 = acos((first_ray_end[2] - agent_pos[2]) / sensor_cutoff_distance);

        double p1 = std::abs(theta2 - theta0) / std::abs(theta2 - theta1);
        double p2 = std::abs(theta1 - theta0) / std::abs(theta2 - theta1);

        double psi1 = atan2(last_ray_end[1] - agent_pos[1],  last_ray_end[0] - agent_pos[0]);
        double psi2 = atan2(first_ray_end[1] - agent_pos[1], first_ray_end[0] - agent_pos[0]);

        // the interpolation of radian may be in 2 directions, the section we care is the 0~180 angle formed by the rays, not the 180~360 part
        // if the <180 angle covers the +x axis, if so, wrap to [-pi, pi]
        if (last_ray_end[0] + first_ray_end[0] - 2 * agent_pos[0] >= 0)
        {
            psi1 = wrap_to_mpi_pi(psi1);
            psi2 = wrap_to_mpi_pi(psi2);
        }
        else
        {
            psi1 = wrap_to_0_2pi(psi1);
            psi2 = wrap_to_0_2pi(psi2);
        }

        double psi0 = p1 * psi1 + p2 * psi2;
        double intersect_vec_x = sensor_cutoff_distance * sin(theta0) * cos(psi0);
        double intersect_vec_y = sensor_cutoff_distance * sin(theta0) * sin(psi0);
        double intersect_vec_z = sensor_cutoff_distance * cos(theta0);

        // ROS_WARN_STREAM("point last is " << last_ray_end[0] << "," << last_ray_end[1] << "," << last_ray_end[2]);
        // ROS_WARN_STREAM("point first is " << first_ray_end[0] << "," << first_ray_end[1] << "," << first_ray_end[2]);
        // ROS_WARN_STREAM("intercept is " << agent_pos[0] + intersect_vec_x << "," << agent_pos[1] + intersect_vec_y << "," << agent_pos[2] + intersect_vec_z);
        std::vector<double> z0_intercept = {agent_pos[0] + intersect_vec_x, agent_pos[1] + intersect_vec_y, agent_pos[2] + intersect_vec_z};
        projected_camera_bounds.push_back(z0_intercept);
    }
    // check if the first ray intersect with ground plane
    if (first_ray_end[2] < 0) // Top left corner of image frame
    {
        double ray_portion = std::abs(agent_pos[2]) / (std::abs(agent_pos[2]) + std::abs(first_ray_end[2]));
        double pos_portion = std::abs(first_ray_end[2]) / (std::abs(agent_pos[2]) + std::abs(first_ray_end[2]));
        double intersect_x = ray_portion * first_ray_end[0] + pos_portion * agent_pos[0];
        double intersect_y = ray_portion * first_ray_end[1] + pos_portion * agent_pos[1];
        double intersect_z = ray_portion * first_ray_end[2] + pos_portion * agent_pos[2];
        std::vector<double> z0_intercept = {intersect_x, intersect_y, intersect_z}; // intersect_z should be 0
        projected_camera_bounds.push_back(z0_intercept);
    }

    for (size_t k = 1; k < q_rotated.size(); ++k)
    {
        std::vector<double> curr_ray_end = sphere_intercept[k];
        std::vector<double> prev_ray_end = sphere_intercept[k - 1];
        if (prev_ray_end[2] * curr_ray_end[2] < 0)
        {
            double theta0 = acos(-agent_pos[2] / sensor_cutoff_distance);
            double theta1 = acos((prev_ray_end[2] - agent_pos[2]) / sensor_cutoff_distance);
            double theta2 = acos((curr_ray_end[2] - agent_pos[2]) / sensor_cutoff_distance);

            double p1 = std::abs(theta2 - theta0) / std::abs(theta2 - theta1);
            double p2 = std::abs(theta1 - theta0) / std::abs(theta2 - theta1);

            double psi1 = atan2(prev_ray_end[1] - agent_pos[1], prev_ray_end[0] - agent_pos[0]);
            double psi2 = atan2(curr_ray_end[1] - agent_pos[1], curr_ray_end[0] - agent_pos[0]);

            if (prev_ray_end[0] + curr_ray_end[0] - 2 * agent_pos[0] >= 0)
            {
                psi1 = wrap_to_mpi_pi(psi1);
                psi2 = wrap_to_mpi_pi(psi2);
            }
            else
            {
                psi1 = wrap_to_0_2pi(psi1);
                psi2 = wrap_to_0_2pi(psi2);
            }

            double psi0 = p1 * psi1 + p2 * psi2;
            double intersect_vec_x = sensor_cutoff_distance * sin(theta0) * cos(psi0);
            double intersect_vec_y = sensor_cutoff_distance * sin(theta0) * sin(psi0);
            double intersect_vec_z = sensor_cutoff_distance * cos(theta0);

        // ROS_WARN_STREAM("psi 1 is " << psi1);
        // ROS_WARN_STREAM("psi 2 is " << psi2);

        // ROS_WARN_STREAM("point prev is " << prev_ray_end[0] - agent_pos[0] << "," << prev_ray_end[1] - agent_pos[1] << "," << prev_ray_end[2] - agent_pos[2]);
        // ROS_WARN_STREAM("point curr is " << curr_ray_end[0] - agent_pos[0] << "," << curr_ray_end[1] - agent_pos[1] << "," << curr_ray_end[2] - agent_pos[2]);
        // ROS_WARN_STREAM("intercept is " << intersect_vec_x << "," << intersect_vec_y << "," << intersect_vec_z);

            std::vector<double> z0_intercept = {agent_pos[0] + intersect_vec_x, agent_pos[1] + intersect_vec_y, agent_pos[2] + intersect_vec_z};
            projected_camera_bounds.push_back(z0_intercept);
        }
        if (curr_ray_end[2] < 0)
        {
            double ray_portion = std::abs(agent_pos[2]) / (std::abs(agent_pos[2]) + std::abs(curr_ray_end[2]));
            double pos_portion = std::abs(curr_ray_end[2]) / (std::abs(agent_pos[2]) + std::abs(curr_ray_end[2]));
            double intersect_x = ray_portion * curr_ray_end[0] + pos_portion * agent_pos[0];
            double intersect_y = ray_portion * curr_ray_end[1] + pos_portion * agent_pos[1];
            double intersect_z = ray_portion * curr_ray_end[2] + pos_portion * agent_pos[2];
            std::vector<double> z0_intercept = {intersect_x, intersect_y, intersect_z}; // intersect_z should be 0
            projected_camera_bounds.push_back(z0_intercept);
        }
    }
    return projected_camera_bounds;
}

std::vector<std::vector<double>> drone_pose_to_projected_camera_bounds(
    const geometry_msgs::Pose &drone_pose,
    const SensorParams &sensor_params,
    int sensor_model_id)
{
    tf2::Quaternion q(
        drone_pose.orientation.x,
        drone_pose.orientation.y,
        drone_pose.orientation.z,
        drone_pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double _, __, yaw;
    m.getRPY(_, __, yaw);

    std::vector<std::vector<double>> q_rotated = rotated_camera_fov(sensor_params, /*roll*/ 0.0, /*sensor_params.pitch*/ sensor_params.pitch, /*yaw*/ yaw);
    // std::cout << "running q_rotated No.15 \n";
    std::vector<double> node_pos = {drone_pose.position.x, drone_pose.position.y, drone_pose.position.z};
    std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pos, q_rotated, sensor_params.max_range[sensor_model_id]);
    return projected_camera_bounds;
}

std::vector<std::vector<double>> drone_pose_to_projected_camera_bounds(
    const nav_msgs::Odometry &drone_pose,
    const SensorParams &sensor_params,
    int sensor_model_id)
{
    tf2::Quaternion q(
        drone_pose.pose.pose.orientation.x,
        drone_pose.pose.pose.orientation.y,
        drone_pose.pose.pose.orientation.z,
        drone_pose.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double _, __, yaw;
    m.getRPY(_, __, yaw);

    std::vector<std::vector<double>> q_rotated = rotated_camera_fov(sensor_params, /*roll*/ 0.0, /*sensor_params.pitch*/ sensor_params.pitch, /*yaw*/ yaw);
    // std::cout << "running q_rotated No.16 \n";
    std::vector<double> node_pos = {drone_pose.pose.pose.position.x, drone_pose.pose.pose.position.y, drone_pose.pose.pose.position.z};
    std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pos, q_rotated, sensor_params.max_range[sensor_model_id]);
    return projected_camera_bounds;
}


SensorParams fetch_sensor_params_from_rosparam_server(ros::NodeHandle &nh)
{
    std::vector<double> max_range;
    int model_count = ros_utils::get_param<double>(nh, "sensor/model_count", 3);
    std::vector<double> temp_max_range;
    nh.param<std::vector<double>>("sensor/max_range", temp_max_range, {610.0, 620.0, 700.0});
    int num_max_range = static_cast<int>(temp_max_range.size());
    if (model_count > num_max_range) {
        ROS_WARN("model_count > max_range.size(). Setting model_count = max_range.size().");
        model_count = num_max_range;
    }
    for (int i = 0; i < model_count; i++)
    {
        max_range.push_back(temp_max_range.at(i));
    }
    // for (int i = 0; i < model_count; ++i)
    // {
    //     std::stringstream ss;
    //     ss << "/sensor/max_range_" << i;
    //     max_range.push_back(ros_utils::get_param<double>(nh, ss.str()));
    // }
    return SensorParams(
        ros_utils::get_param<double>(nh, "sensor/focal_length", 15.0),
        ros_utils::get_param<double>(nh, "sensor/width", 10.0),
        ros_utils::get_param<double>(nh, "sensor/height", 10.0),
        ros_utils::get_param<double>(nh, "sensor/pitch", 0.785398),
        max_range,
        model_count
    );
}