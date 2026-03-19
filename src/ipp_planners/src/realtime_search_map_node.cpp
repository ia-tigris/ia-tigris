// #include "ipp_planners/ipp_planners_node.h"
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <planner_map_interfaces/Plan.h>
#include <planner_map_interfaces/PlanRequest.h>
#include <planner_map_interfaces/Waypoint.h>
#include <planner_map_interfaces/ImageWithPose.h>

#include "planner_map_interfaces/ros_utils.h"
#include "planner_map_interfaces/camera_projection.h"
#include "ipp_planners/SearchMapSetup.h"
#include "ipp_planners/SearchMapUpdate.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/String.h" 
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"
#include "ipp_planners/Tigris.h"
#include "ipp_planners/visualizations.h"
#include "math_utils/math_utils.h"
#include "ipp_planners/SearchMap.h"
#include <chrono>
#include <string>

/* ---- ROS STUFF ---- */
bool visualize;
std::string sim = "false";
bool no_info_on_turns = false;
bool pub_search_map_metrics;
std::string local_frame = "local_enu";
double node_loop_hz;
int search_belief_publishing_every_n_loops;
ros::Subscriber waypoint_num_sub;
ros::Subscriber final_path_sub;

/* ---- TOPIC NAMES ---- */
std::string agent_topic;
std::string agent_cam_topic;
std::string image_with_cam_pose_topic;

/* ---- STATE ATTRIBUTES ---- */
int waypoint_num = 0; //waypoint that we're MOVING TO
bool prev_path_waypoint_init = false; //use bool instead of checking each value in msg cuz it's cleaner
geometry_msgs::Pose prev_path_waypoint;
geometry_msgs::PoseStamped agent_pose;
bool received_first_planned_path = false;
bool received_first_waypoint_num = false;
geometry_msgs::PoseStamped agent_cam_pose;
planner_map_interfaces::Plan current_plan_msg;
planner_map_interfaces::ImageWithPose image_with_pose;
bool received_new_agent_odom = false;
bool received_new_agent_cam_state = false;
bool received_new_image_with_pose = false;
bool use_cam_states_not_agent_state = false;

/* ---- MAP ATTRIBUTES ---- */
geometry_msgs::Polygon global_search_bounds;
SearchMap search_map;
int x_start = INT_MAX;
int x_end = INT_MIN;
int y_start = INT_MAX;
int y_end = INT_MIN;
int map_resolution;
int map_resolution_from_config;
bool use_ratio_distance_params = false;
int map_resolution_num;
double confidence_threshold = 0.5;
float ground_offset = 0;
Eigen::Vector3d ground_normal{0, 0, 1}; // Should point up from ground - if pointing into ground, will cause errors

bool first_setup_has_occurred = false;
bool check_grid_size = false;

/* ---- SENSOR ATTRIBUTES ---- */
SensorParams sensor_params;
float fx = 338.136183;
float fy = 336.039570;
float cx = 160.829;
float cy = 112.614;
Eigen::Matrix3d K_inv;

/* ---- VISUALIZATION ATTRIBUTES ---- */ 
double visualization_alpha=0.2;

/**
 * @brief Get the circumscribing rectangle points of a polygon
 *
 * @param polygon_bounds
 * @return std::vector<double> x_start, y_start, x_end, y_end
 */
std::vector<double> get_circumscribing_rectangle(std::vector<std::vector<double>> &polygon_bounds)
{
    // get circumscribing rectangle search bound over polygonal bounds
    double x_start = INT_MAX;
    double y_start = INT_MAX;
    double x_end = INT_MIN;
    double y_end = INT_MIN;

    // shrink the rectangular bounds to circumscribe the polygonal bounds
    for (int i = 0; i < polygon_bounds.size(); ++i)
    {
        int x = static_cast<int>(polygon_bounds[i][0]);
        int y = static_cast<int>(polygon_bounds[i][1]);
        if (x < x_start)
            x_start = x;
        if (x > x_end)
            x_end = x;
        if (y < y_start)
            y_start = y;
        if (y > y_end)
            y_end = y;
    }
    return {x_start, y_start, x_end, y_end};
}

double find_width_of_sensor_at_height(double flight_height)
{
    std::vector<std::vector<double>> q_rotated = rotated_camera_fov(sensor_params, 0.0, sensor_params.pitch, 0.0);
    ROS_INFO_STREAM("pitch: " << sensor_params.pitch);
    std::vector<double> node_pose = {0.0, 0.0, flight_height, 0.0};
    ROS_INFO_STREAM("Finding width of sensor at height " << flight_height);
    ROS_INFO_STREAM("max range: " << sensor_params.highest_max_range);
    std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pose, q_rotated, sensor_params.highest_max_range);

    for (size_t i = 0; i < projected_camera_bounds.size(); ++i)
    {
        ROS_INFO_STREAM("Projected camera bounds::" << projected_camera_bounds[i][0] << "," << projected_camera_bounds[i][1]);
    }

    double shortest_edge_length;
    size_t number_of_edge = projected_camera_bounds.size();
    // edge projected_camera_bounds [0] and [3 or 4]
    double dx_0 = projected_camera_bounds[0][0] - projected_camera_bounds[number_of_edge - 1][0];
    double dy_0 = projected_camera_bounds[0][1] - projected_camera_bounds[number_of_edge - 1][1];
    shortest_edge_length = sqrt(dx_0 * dx_0 + dy_0 * dy_0);
    for (size_t edge = 0; edge < projected_camera_bounds.size() - 1; ++edge)
    {
        double dx_i = projected_camera_bounds[edge][0] - projected_camera_bounds[edge + 1][0];
        double dy_i = projected_camera_bounds[edge][1] - projected_camera_bounds[edge + 1][1];
        double edge_length_i = sqrt(dx_i * dx_i + dy_i * dy_i);
        if (edge_length_i < shortest_edge_length)
        {
            shortest_edge_length = edge_length_i;
        }
    }
    ROS_INFO_STREAM("Shortest edge length: " << shortest_edge_length);
    return shortest_edge_length;
}

/**
 * @brief updates priors, and if search bounds have changed, resets the map
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
bool setupMapService(ipp_planners::SearchMapSetup::Request &req,
                     ipp_planners::SearchMapSetup::Response &res)
{
    ROS_INFO("[SearchMap] Received new search map setup request");
    // read in the polygon  bounds
    global_search_bounds = req.search_bounds;
    std::vector<std::vector<double>> polygon_bounds = extract_vector_polygon_bounds_from_polygon_msg(req.search_bounds);

    // Version that sets the map resolution based on the longest edge of the polygon
    // if (use_ratio_distance_params)
    // {
    //     double max_edge_size = std::max(abs(x_end - x_start), abs(y_end - y_start));
    //     map_resolution = max_edge_size / map_resolution_num;
    //     ROS_DEBUG_STREAM("Update proportional map resolution to::" << map_resolution);
    // }

    // Version that uses map_resolution_num as the mas number of cells on the bottom edge of the camera footprint
    if (use_ratio_distance_params)
    {
        ROS_INFO_STREAM("Using proportional map resolution");
        double sensor_width = find_width_of_sensor_at_height(req.flight_height);
        int new_map_resolution = sensor_width / map_resolution_num;
        map_resolution = std::max(new_map_resolution, map_resolution_from_config);
        ROS_INFO_STREAM("Sensor width: " << sensor_width);
        ROS_INFO_STREAM("New map resolution: " << new_map_resolution);
        ROS_DEBUG_STREAM("Update proportional map resolution to: " << map_resolution);
    }

    // check if we need to reset the map due to changed search bounds
    int x_start_old = x_start;
    int y_start_old = y_start;
    int x_end_old = x_end;
    int y_end_old = y_end;
    int map_resolution_old = map_resolution;

    auto rect_bounds = get_circumscribing_rectangle(polygon_bounds);
    x_start = rect_bounds[0];
    y_start = rect_bounds[1];
    x_end = rect_bounds[2];
    y_end = rect_bounds[3];

    if (!first_setup_has_occurred || x_start != x_start_old || y_start != y_start_old ||
        x_end != x_end_old || y_end != y_end_old ||
        map_resolution != map_resolution_old)
    {
        // resets the map if any of the search bounds parameters have changed
        // TODO: figure out how to save information from previous search query
        ROS_INFO("[SearchMap] Initializing or reseting the search map");
        std::vector<double> empty = {};
        search_map = SearchMap(polygon_bounds, x_start, y_start, x_end, y_end, map_resolution,
                               confidence_threshold, sensor_params, empty);
        first_setup_has_occurred = true;
    }

    // update target priors
    search_map.prior_list.clear(); // clear old priors
    if (req.target_priors.size() > 0)
    {
        for (auto &new_prior : req.target_priors)
        {
            if (new_prior.grid_prior.bounds.points.size() != 0)
            {
                search_map.prior_list.push_back(new_prior.grid_prior);
            }
        }
    }
    search_map.update_priors();

    res.setup_response = true;
    ROS_INFO("[SearchMap] Map reset_map successful");
    return true;
}

bool updateMapService(ipp_planners::SearchMapUpdate::Request &req,
                      ipp_planners::SearchMapUpdate::Response &res)
{
    if (!first_setup_has_occurred)
    {
        return false;
    }

    res.search_bounds = global_search_bounds;

    res.x_start = x_start;
    res.y_start = y_start;
    res.x_end = x_end;
    res.y_end = y_end;
    res.map_resolution = map_resolution;
    res.confidence_threshold = confidence_threshold;

    int flattened_size = search_map.map.size() * search_map.map[0].size();
    std::vector<double> flattened_map(flattened_size, 0.0);
    std::vector<double> flat_priority_map(flattened_size);
    std::vector<int> flat_model_map(flattened_size);
    int k = 0;
    for (int i = 0; i < search_map.map.size(); i++)
    {
        for (int j = 0; j < search_map.map[0].size(); j++)
        {
            k = i * search_map.map[0].size() + j;
            flattened_map[k] = search_map.map.at(i).at(j);
            flat_priority_map[k] = search_map.priority.at(i).at(j);
            flat_model_map[k] = search_map.sensor_model_id.at(i).at(j);
        }
    }
    res.map_values = flattened_map;
    res.priority_map = flat_priority_map;
    res.sensor_model_id_map = flat_model_map;
    return true;
}

bool check_camera_projection_bigger_than_grid_cell(std::vector<std::vector<double>> projected_camera_bounds)
{
    // check the size of a grid cell, if the grid cell is bigger than the camera fov, then the grid might be too big
    if (projected_camera_bounds.size() <= 1)
    {
        ROS_WARN_STREAM("Camera footprint not complete (camera range probably too small) Skipping map update");
        return false;
    }

    if (!check_grid_size)
    {
        check_grid_size = true;
        
        double shortest_edge_length;
        size_t number_of_edge = projected_camera_bounds.size();
        // edge projected_camera_bounds [0] and [3 or 4]
        double dx_0 = projected_camera_bounds[0][0] - projected_camera_bounds[number_of_edge - 1][0];
        double dy_0 = projected_camera_bounds[0][1] - projected_camera_bounds[number_of_edge - 1][1];
        shortest_edge_length = sqrt(dx_0 * dx_0 + dy_0 * dy_0);
        for (size_t edge = 0; edge < projected_camera_bounds.size() - 1; ++edge)
        {
            double dx_i = projected_camera_bounds[edge][0] - projected_camera_bounds[edge + 1][0];
            double dy_i = projected_camera_bounds[edge][1] - projected_camera_bounds[edge + 1][1];
            double edge_length_i = sqrt(dx_i * dx_i + dy_i * dy_i);
            if (edge_length_i < shortest_edge_length)
            {
                shortest_edge_length = edge_length_i;
            }
        }
        if (shortest_edge_length < (double)map_resolution)
        {
            ROS_ERROR_STREAM("The grid size might be too big, bigger than the shortest edge of sensor footprint.");
        }
    }
    return true;
}

void compute_bayes_update_no_image(std::vector<double> node_pose, std::vector<std::vector<double>> projected_camera_bounds, std::vector<int> bbox)
{
    // Compute drone's affect on the real time map
    // make a map copy s.t. we can compute belief difference deltas for reward counting
    search_map.copy_map = search_map.map;
    double total_reward = 0.0;

    for (int i = bbox[0]; i <= bbox[2]; ++i)
    {
        for (int j = bbox[1]; j <= bbox[3]; ++j)
        {
            // check if cell lies completely inside camera footprint
            if (search_map.check_cell_inside_convex_poly(i, j, projected_camera_bounds))
            {
                ROS_INFO_THROTTLE(10, "A cell was registered to be within the footprint");
                int x = i * search_map.map_resolution + search_map.x_start;
                int y = j * search_map.map_resolution + search_map.y_start;
                // find distance from node pos to centroid of camera footprint
                double range = std::sqrt(std::pow((x - node_pose[0]), 2.0) + std::pow((y - node_pose[1]), 2.0) + std::pow((0.0 - node_pose[2]), 2.0));
                // calculate bayes update
                double prior = search_map.copy_map.at(i).at(j);
                if (search_map.copy_map.at(i).at(j) > confidence_threshold)
                {
                    search_map.copy_map.at(i).at(j) =
                        sensor_params.tpr(range, search_map.sensor_model_id.at(i).at(j)) * search_map.copy_map.at(i).at(j)    /**/
                        / (sensor_params.tpr(range, search_map.sensor_model_id.at(i).at(j)) * search_map.copy_map.at(i).at(j) /**/
                           + sensor_params.fpr(range, search_map.sensor_model_id.at(i).at(j)) * (1 - search_map.copy_map.at(i).at(j)));
                }
                else
                {
                    search_map.copy_map.at(i).at(j) = sensor_params.fnr(range, search_map.sensor_model_id.at(i).at(j)) * search_map.copy_map.at(i).at(j)      /**/
                                                      / ((sensor_params.fnr(range, search_map.sensor_model_id.at(i).at(j))) * search_map.copy_map.at(i).at(j) /**/
                                                         + (sensor_params.tnr(range, search_map.sensor_model_id.at(i).at(j))) * (1 - search_map.copy_map.at(i).at(j)));
                }
                double belief_diff = search_map.copy_map.at(i).at(j) - prior;
                ROS_INFO_STREAM_THROTTLE(10, "Belief before: " << prior << " Belief after: " << search_map.copy_map.at(i).at(j));
                if (belief_diff > 0)
                    total_reward += belief_diff * search_map.Rs;
                else
                    total_reward += abs(belief_diff) * search_map.Rf;
            }
        }
    }
    search_map.map = search_map.copy_map;
    // std::cout << "Pose reward: " << total_reward << std::endl;
}
void agent_odom_callback(const nav_msgs::Odometry &msg){
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    agent_pose = pose;
    received_new_agent_odom = true;
}
void updateAgentCamStateCallback(const nav_msgs::Odometry &msg)
{
    // ROS_DEBUG_STREAM("Received agent pose");
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    agent_cam_pose = pose;
    // agent_cam_pose = msg;

    // ROS_DEBUG_STREAM("New agent pose at x::" << agent_pose.pose.position.x << " y::" << agent_pose.pose.position.y << " z::" << agent_pose.pose.position.z);
    received_new_agent_cam_state = true;
    use_cam_states_not_agent_state = true;
}

void updateImageWithCamPoseCallback(const planner_map_interfaces::ImageWithPose &msg)
{
    ROS_DEBUG_STREAM("Received image with pose");
    image_with_pose = msg;
    received_new_image_with_pose = true;
}

// calculate search map metrics
std::vector<double> calc_search_map_metrics(SearchMap &map_repr)
{
    double total_entropy = 0.0;
    double total_probs = 0.0;
    double curr_value = 0.0;
    std::vector<double> map_metrics;
    // loop through once element by element and calculate all metrics
    for (int i = 0; i < map_repr.num_rows; i++)
    {
        for (int j = 0; j < map_repr.num_cols; j++)
        {
            curr_value = map_repr.map.at(i).at(j);
            total_entropy += map_repr.calc_entropy(curr_value);
            total_probs += curr_value;
        }
    }
    map_metrics.push_back(total_entropy/(map_repr.num_rows * map_repr.num_cols));
    map_metrics.push_back(total_probs/(map_repr.num_rows * map_repr.num_cols));
    return map_metrics;
}

bool is_turning(double eps = 0.0001)
{
    geometry_msgs::Pose curr_waypoint, prev_waypoint;

    //calculate the current and previous waypoint nums
    curr_waypoint = current_plan_msg.plan[waypoint_num].position;
    if(waypoint_num == 0 && !prev_path_waypoint_init)
    {
        return false; //first plan
    }
    else if(waypoint_num == 0 && prev_path_waypoint_init)
    {
        prev_waypoint = prev_path_waypoint; //replanning -> grab prev waypoint from prev path
    }
    else
    {   
        prev_waypoint = current_plan_msg.plan[waypoint_num-1].position;
    }

    //convert quaternion to rpy for each waypoint
    tf2::Quaternion curr_quat(curr_waypoint.orientation.x, curr_waypoint.orientation.y,
                                    curr_waypoint.orientation.z, curr_waypoint.orientation.w);
    tf2::Matrix3x3 curr_orient_mat(curr_quat);
    double curr_roll, curr_pitch, curr_yaw;
    curr_orient_mat.getRPY(curr_roll, curr_pitch, curr_yaw);
    tf2::Quaternion prev_quat(prev_waypoint.orientation.x, prev_waypoint.orientation.y,
                                prev_waypoint.orientation.z, prev_waypoint.orientation.w);
    tf2::Matrix3x3 prev_orient_mat(prev_quat);
    double prev_roll, prev_pitch, prev_yaw;
    prev_orient_mat.getRPY(prev_roll, prev_pitch, prev_yaw);

    //calculate direction of waypoint segment
    double dir = std::abs(ang::TurnDirection(prev_yaw, curr_yaw));

    //save off previous waypoint in case we get a replan (and waypoint num count resets)
    prev_path_waypoint = prev_waypoint;
    prev_path_waypoint_init = true;

    //check to see if we're turning or not --> if dir above threshold then yes otherwise return false
    return dir > eps;
}

void waypoint_num_topic_callback(const std_msgs::UInt32 &msg)
{
    waypoint_num = msg.data;
    received_first_waypoint_num = true;
}

void final_path_callback(const planner_map_interfaces::Plan &msg)
{
    ROS_WARN_STREAM("[Realtime Search Map] Recieved a new plan!");
    current_plan_msg = msg;
    received_first_planned_path = true;

}

int main(int argc, char **argv)
{
    K_inv << 1.0 / fx, 0.0, -cx / fx,
        0.0, 1.0 / fy, -cy / fy,
        0.0, 0.0, 1.0;
    ros::init(argc, argv, "search_map_update_server");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
    ros::Time debug_time_projection = ros::Time::now();

    agent_topic = ros_utils::get_param<std::string>(pnh, "agent_odom_topic", "/simulator/agent_pose");
    agent_cam_topic = ros_utils::get_param<std::string>(pnh, "agent_cam_pose_topic", "/simulator/agent_cam_pose");
    sensor_params = fetch_sensor_params_from_rosparam_server(nh);
    map_resolution = ros_utils::get_param<double>(pnh, "map_resolution", 10);
    map_resolution_from_config = map_resolution;
    use_ratio_distance_params = ros_utils::get_param<bool>(pnh, "use_ratio_distance_params", false);
    map_resolution_num = ros_utils::get_param<int>(pnh, "map_resolution_num", 100);
    confidence_threshold = ros_utils::get_param<double>(pnh, "confidence_threshold", 0.5);
    node_loop_hz = ros_utils::get_param<double>(pnh, "node_loop_hz", 5);
    visualize = ros_utils::get_param<bool>(pnh, "visualization", true);
    search_belief_publishing_every_n_loops = ros_utils::get_param<int>(pnh, "search_belief_publishing_every_n_loops", 1);
    visualization_alpha = ros_utils::get_param<double>(pnh, "visualization_alpha", 0.5);
    pub_search_map_metrics = ros_utils::get_param<bool>(pnh, "pub_search_map_metrics", false);

    //check if we're running simple sim or not
    sim = ros_utils::get_param<std::string>(pnh, "sim", "false");
    no_info_on_turns = ros_utils::get_param<bool>(pnh, "no_info_on_turns", false);
    if(no_info_on_turns && sim == "simple")
    {
        ROS_WARN_STREAM("Ignoring info gain on turns because sim is simple and no_info_on_turns is set to true.");

        //subscribe to waypoint num to tell if we're on a curved segment or not
        waypoint_num_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "waypoint_num_topic"), 1, waypoint_num_topic_callback);
        final_path_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "plan_output_topic"), 1, final_path_callback);
    }

    ros::Subscriber agent_state_sub = nh.subscribe(agent_topic, 1, agent_odom_callback);
    ros::Subscriber agent_cam_state_sub = nh.subscribe(agent_cam_topic, 1, updateAgentCamStateCallback);

    if (pnh.getParam("image_with_cam_pose_topic", image_with_cam_pose_topic)){
        ros::Subscriber image_with_cam_pose_sub = nh.subscribe(image_with_cam_pose_topic, 1, updateImageWithCamPoseCallback);
    }

    ros::Publisher vehicle_pose_pub = nh.advertise<visualization_msgs::Marker>("realtime_search/markers/agent_pose", 10);
    ros::Publisher search_belief_vis_pub = nh.advertise<visualization_msgs::Marker>("search_belief", 10);
    ros::Publisher average_entropy_pub = nh.advertise<std_msgs::Float32>("realtime_search/average_entropy", 10);
    ros::Publisher average_probs_pub = nh.advertise<std_msgs::Float32>("realtime_search/average_prob", 10);

    ros::ServiceServer service = pnh.advertiseService("search_map_setup", setupMapService);
    ros::ServiceServer service2 = pnh.advertiseService("search_map_update", updateMapService);

    ros::Rate loop_rate(node_loop_hz);
    int loop_counter = 0;
    while (ros::ok())
    {
        // do nothing until we've setup
        if (!first_setup_has_occurred)
        {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        else if (pub_search_map_metrics)
        {
            std::vector<double> search_map_metrics = calc_search_map_metrics(search_map);

            std_msgs::Float32 avg_entropy_msg;
            avg_entropy_msg.data = search_map_metrics.at(0);
            std_msgs::Float32 avg_probs_msg;
            avg_probs_msg.data = search_map_metrics.at(1);
        
            average_entropy_pub.publish(avg_entropy_msg);
            average_probs_pub.publish(avg_probs_msg);
        }

        tf::Quaternion q(
            agent_pose.pose.orientation.x,
            agent_pose.pose.orientation.y,
            agent_pose.pose.orientation.z,
            agent_pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if (received_new_image_with_pose)
        {
            // ROS_DEBUG_STREAM("Using image with pose");
            received_new_image_with_pose = false;
            Eigen::Quaterniond cam_quat;
            tf::quaternionMsgToEigen(image_with_pose.pose.orientation, cam_quat);
            // ROS_DEBUG_STREAM("Camera orientation: " << image_with_pose.pose.orientation.x << " " << image_with_pose.pose.orientation.y << " " << image_with_pose.pose.orientation.z << " " << image_with_pose.pose.orientation.w);
            // ROS_DEBUG_STREAM("cam_quat:" << cam_quat.vec() << " " << cam_quat.w());

            Eigen::Matrix3d pixelToRay = cam_quat.matrix() * K_inv;
            double x, y, z;
            x = image_with_pose.pose.position.x, y = image_with_pose.pose.position.y, z = image_with_pose.pose.position.z;
            Eigen::Vector3d camCenter(x, y, z);
            // Eigen::Vector3d camCenter(0, 0, 30);
            // ROS_DEBUG_STREAM("FIRST agent_pose.pose.position.x=" << agent_pose.pose.position.x << ", agent_pose.pose.position.y=" << agent_pose.pose.position.y << ", agent_pose.pose.position.z=" << agent_pose.pose.position.z);
            // ROS_DEBUG_STREAM("x=" << x << ", y=" << y << ", z=" << z);
            // ROS_DEBUG_STREAM("FIRST camCenter" << camCenter);

            float camCenterGroundDist = ground_offset - ground_normal.dot(camCenter);

            // std::cout << "Projecting and filtering!" << std::endl;
            // ROS_DEBUG_STREAM("Image dimensions: w=" << image_with_pose.image.width << ", h=" << image_with_pose.image.height);

            search_map.copy_map = search_map.map;
            for (size_t i = 0; i < image_with_pose.image.width; i+=(size_t)(image_with_pose.image.width / 50))
            {
                for (size_t j = 0; j < image_with_pose.image.height; j+=(size_t)(image_with_pose.image.width / 50))
                {
                    // ROS_DEBUG_STREAM("pixelToRay: " << pixelToRay);
                    Eigen::Vector3d pixelHomogenous{static_cast<double>(i), static_cast<double>(j), 1};
                    // ROS_DEBUG_STREAM("pixelHomogeneous: " << pixelHomogenous);
                    Eigen::Vector3d rayDir = pixelToRay * pixelHomogenous;
                    // ROS_DEBUG_STREAM("Ray direction: " << rayDir);

                    float rayPerpendicularComponent = ground_normal.dot(rayDir);
                    // ROS_DEBUG_STREAM("rayPerpendicularComponent=" << rayPerpendicularComponent);
                    if (rayPerpendicularComponent >= 0)
                    {
                        // ROS_DEBUG_STREAM("skipping because rayPerpendicularComponent=" << rayPerpendicularComponent);
                        continue;
                    }
                    float rayLambda = camCenterGroundDist / rayPerpendicularComponent;

                    // ROS_DEBUG_STREAM("agent_pose.pose.position.x=" << agent_pose.pose.position.x << ", agent_pose.pose.position.y=" << agent_pose.pose.position.y << ", agent_pose.pose.position.z=" << agent_pose.pose.position.z);
                    // ROS_DEBUG_STREAM("camCenter" << camCenter);
                    Eigen::Vector3d intersect = camCenter + rayLambda * rayDir;
                    // ROS_DEBUG_STREAM("Intersect: " << intersect);

                    if (intersect(0) < x_start || intersect(0) > x_end || intersect(1) < y_start ||  intersect(1) > y_end )
                    {
                        // ROS_DEBUG_STREAM("Skipping because intersect" << intersect(0) << "," << intersect(1) << " is out of bounds x:[" << x_start << "," << x_end << "], y:[" << y_start << "," << y_end << "]");
                        continue;
                    }

                    size_t gridRow = (size_t)((intersect(0) - x_start) / map_resolution);
                    size_t gridCol = (size_t)((intersect(1) - y_start) / map_resolution);
                    // find distance from node pos to centroid of camera footprint
                    double range = std::sqrt(std::pow((intersect(0) - agent_pose.pose.position.x), 2.0) + std::pow((intersect(1) - agent_pose.pose.position.y), 2.0) + std::pow((0.0 - agent_pose.pose.position.z), 2.0));
                    // ROS_DEBUG_STREAM("Range=" << range);

                    uint8_t pixelValue = image_with_pose.image.data[i + j * image_with_pose.image.width];
                    double prior = search_map.map.at(gridRow).at(gridCol);

                    auto &sensor_model_id_at_grid = search_map.sensor_model_id.at(gridRow).at(gridCol);
                    if (range > sensor_params.max_range.at(sensor_model_id_at_grid))
                    {
                        // ROS_DEBUG_STREAM("Skipping because range=" << range << " is greater than max_range=" << sensor_params.max_range.at(sensor_model_id_at_grid));
                        continue;
                    }
                    // ROS_DEBUG_STREAM("Updating map at index (row=" << gridRow << ",col=" << gridCol << ") with prior=" << prior << " with pixelValue=" << (int)pixelValue);
                    if (pixelValue == 255)
                    {
                        auto tpr = sensor_params.tpr(range, sensor_model_id_at_grid) * prior;
                        auto fpr = sensor_params.fpr(range, sensor_model_id_at_grid) * (1 - prior);
                        // ROS_DEBUG_STREAM("tpr=" << tpr << ", fpr=" << fpr);
                        auto p_obs_pos = tpr + fpr;
                        // ROS_DEBUG_STREAM("p_obs_pos=" << p_obs_pos);
                        search_map.copy_map.at(gridRow).at(gridCol) = std::max(0.01, std::min(tpr / p_obs_pos, 0.99));
                    }
                    else if (pixelValue == 0)
                    {
                        auto fnr = sensor_params.fnr(range, sensor_model_id_at_grid) * prior;
                        auto tnr = sensor_params.tnr(range, sensor_model_id_at_grid) * (1 - prior);
                        // ROS_DEBUG_STREAM("fnr=" << fnr << ", tnr=" << tnr);
                        auto p_obs_neg = fnr + tnr;
                        // ROS_DEBUG_STREAM("p_obs_neg=" << p_obs_neg);
                        search_map.copy_map.at(gridRow).at(gridCol) = std::max(0.01, std::min(fnr / p_obs_neg, 0.99));
                    }
                    else
                    {
                        ROS_WARN_STREAM("Unhandled pixel value: " << (int) pixelValue);
                    }
                    // ROS_DEBUG_STREAM("New cell prob: " << search_map.copy_map.at(i).at(j));
                    if (search_map.copy_map.at(gridRow).at(gridCol) > 1.0)
                    {
                        ROS_WARN_STREAM("New cell prob is greater than 1.0: " << search_map.copy_map.at(gridRow).at(gridCol));
                    }
                }
            }
            search_map.map = search_map.copy_map;
        }
        else if (received_new_agent_odom || use_cam_states_not_agent_state) // will add received_new_agent_cam_state later
        {
            if (use_cam_states_not_agent_state && received_new_agent_cam_state){
                ROS_WARN_THROTTLE(10, "Updating search map from cam state");
                received_new_agent_cam_state = false;

                tf::Quaternion q(
                    agent_cam_pose.pose.orientation.x,
                    agent_cam_pose.pose.orientation.y,
                    agent_cam_pose.pose.orientation.z,
                    agent_cam_pose.pose.orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                double combined_pitch = pitch; // + search_map.sensor_params.pitch;
                // std::cout << "Updating search map" << std::endl;
                // std::cout << "Agent Position: " << agent_pose.pose.position.x << ", " << agent_pose.pose.position.y << ", " << agent_pose.pose.position.z << std::endl;
                std::vector<std::vector<double>> q_rotated = rotated_camera_fov(search_map.sensor_params, /*roll*/ roll, /*pitch*/ combined_pitch, /*yaw*/ yaw);
                std::vector<double> node_pose = {agent_pose.pose.position.x, agent_pose.pose.position.y, agent_pose.pose.position.z, yaw};
                std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pose, q_rotated, sensor_params.highest_max_range);
                std::vector<int> bbox = search_map.find_bounding_box(projected_camera_bounds);
                double marker_scale = ros_utils::get_param<double>(pnh, "visualization_scale", 1.0);

                bool result = check_camera_projection_bigger_than_grid_cell(projected_camera_bounds);
                if (!result)
                {
                    ros::spinOnce();
                    loop_rate.sleep();
                    continue;
                }
                
                double alpha = 1.0;
                if (!no_info_on_turns || sim != "simple" || !received_first_waypoint_num || !received_first_planned_path || !is_turning())
                {
                    compute_bayes_update_no_image(node_pose, projected_camera_bounds, bbox);
                }
                else
                {
                    alpha = 0.2;
                }


                // To debug why camera projections are different when run locally vs on hardware
                // Also to debug why the map is not updating
                ros::Time current_time = ros::Time::now();
                if ((current_time - debug_time_projection).toSec() > 10)
                {
                    debug_time_projection = current_time;
                    ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
                    ROS_INFO("Sensor Param Pitch: %f, Combined Pitch: %f", search_map.sensor_params.pitch, combined_pitch);
                    
                    ROS_INFO("q_rotated: ");
                    for (size_t i = 0; i < q_rotated.size(); i++)
                    {
                        ROS_INFO_STREAM("[" << q_rotated[i][0] << ", " << q_rotated[i][1] << ", " << q_rotated[i][2] << "]");
                    }
                    ROS_INFO("Node Pose: ");
                    ROS_INFO_STREAM("[" << node_pose[0] << ", " << node_pose[1] << ", " << node_pose[2] << ", " << node_pose[3] << "]");
                    ROS_INFO_STREAM("Max Range: " << sensor_params.highest_max_range);
                    ROS_INFO_STREAM("Projected camera bounds: ");
                    for (size_t i = 0; i < projected_camera_bounds.size(); i++)
                    {
                        ROS_INFO_STREAM("[" << projected_camera_bounds[i][0] << ", " << projected_camera_bounds[i][1] << ", " << projected_camera_bounds[i][2] << "]");
                    }
                    ROS_INFO_STREAM("Bounding Box: ");
                    ROS_INFO_STREAM("[" << bbox[0] << ", " << bbox[1] << ", " << bbox[2] << ", " << bbox[3] << "]");
                }

                // publish the frustum
                if (visualize)
                {
                    vehicle_pose_pub.publish(visualize_frustum(agent_pose.pose.position.x,
                                                            agent_pose.pose.position.y,
                                                            agent_pose.pose.position.z,
                                                            yaw,
                                                            combined_pitch,
                                                            roll,
                                                            search_map.sensor_params,
                                                            local_frame,
                                                            marker_scale,
                                                            alpha));
                }
            }
            else if (!use_cam_states_not_agent_state && received_new_agent_odom){
                ROS_INFO_THROTTLE(10, "Updating search map from agent odom");
                received_new_agent_odom = false;
                double combined_pitch = pitch + search_map.sensor_params.pitch;
                // std::cout << "Updating search map" << std::endl;
                // std::cout << "Agent Position: " << agent_pose.pose.position.x << ", " << agent_pose.pose.position.y << ", " << agent_pose.pose.position.z << std::endl;
                std::vector<std::vector<double>> q_rotated = rotated_camera_fov(search_map.sensor_params, /*roll*/ roll, /*pitch*/ combined_pitch, /*yaw*/ yaw);
                std::vector<double> node_pose = {agent_pose.pose.position.x, agent_pose.pose.position.y, agent_pose.pose.position.z, yaw};
                std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pose, q_rotated, sensor_params.highest_max_range);
                std::vector<int> bbox = search_map.find_bounding_box(projected_camera_bounds);
                double marker_scale = ros_utils::get_param<double>(pnh, "visualization_scale", 1.0);

                bool result = check_camera_projection_bigger_than_grid_cell(projected_camera_bounds);
                if (!result)
                {
                    ros::spinOnce();
                    loop_rate.sleep();
                    continue;
                }
                
                double alpha = 1.0;
                if (!no_info_on_turns || sim != "simple" || !received_first_waypoint_num || !received_first_planned_path || !is_turning())
                {
                    compute_bayes_update_no_image(node_pose, projected_camera_bounds, bbox);
                }
                else
                {
                    alpha = 0.2;
                }


                // To debug why camera projections are different when run locally vs on hardware
                // Also to debug why the map is not updating
                ros::Time current_time = ros::Time::now();
                if ((current_time - debug_time_projection).toSec() > 10)
                {
                    debug_time_projection = current_time;
                    ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
                    ROS_INFO("Sensor Param Pitch: %f, Combined Pitch: %f", search_map.sensor_params.pitch, combined_pitch);
                    
                    ROS_INFO("q_rotated: ");
                    for (size_t i = 0; i < q_rotated.size(); i++)
                    {
                        ROS_INFO_STREAM("[" << q_rotated[i][0] << ", " << q_rotated[i][1] << ", " << q_rotated[i][2] << "]");
                    }
                    ROS_INFO("Node Pose: ");
                    ROS_INFO_STREAM("[" << node_pose[0] << ", " << node_pose[1] << ", " << node_pose[2] << ", " << node_pose[3] << "]");
                    ROS_INFO_STREAM("Max Range: " << sensor_params.highest_max_range);
                    ROS_INFO_STREAM("Projected camera bounds: ");
                    for (size_t i = 0; i < projected_camera_bounds.size(); i++)
                    {
                        ROS_INFO_STREAM("[" << projected_camera_bounds[i][0] << ", " << projected_camera_bounds[i][1] << ", " << projected_camera_bounds[i][2] << "]");
                    }
                    ROS_INFO_STREAM("Bounding Box: ");
                    ROS_INFO_STREAM("[" << bbox[0] << ", " << bbox[1] << ", " << bbox[2] << ", " << bbox[3] << "]");
                }

                // publish the frustum
                if (visualize)
                {
                    vehicle_pose_pub.publish(visualize_frustum(agent_pose.pose.position.x,
                                                            agent_pose.pose.position.y,
                                                            agent_pose.pose.position.z,
                                                            yaw,
                                                            combined_pitch,
                                                            roll,
                                                            search_map.sensor_params,
                                                            local_frame,
                                                            marker_scale,
                                                            alpha));
                }
            }
        }
        // publish search metrics for testing

        // visualize the map itself
        if (visualize && loop_counter % search_belief_publishing_every_n_loops == 0)
        {
            search_belief_vis_pub.publish(visualize_map(search_map, false, "local_enu", visualization_alpha));
            loop_counter = 0;
        }

        loop_counter++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}