
#include <algorithm>
#include <cstdint>
#include <ros/ros.h>
#include <memory>
#include <vector>
#include <iostream>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cmath>
#include <string>
#include <vector>

#include "planner_map_interfaces/ros_utils.h"

#include <ipp_belief/ParticleFiltersBelief.h>
#include <ipp_belief/AddTargetPriors.h>

// source code includes
#include "ipp_belief/belief_manager.h"
#include "ipp_belief/trackers.h"
#include "ipp_belief/colors.h"
#include "ipp_belief/control.h"
#include "ipp_belief/observation.h"
#include "ipp_belief/information.h"
#include "ipp_belief/visualize.h"

#include <visualization_msgs/MarkerArray.h>

using namespace tracking;

ros::Publisher particle_pub;
double marker_scale;
double marker_alpha_multiplier;
visualization_msgs::MarkerArray ma;
std::unordered_map<unsigned int, std::string> class_id_to_label;

visualization_msgs::Marker make_global_id_text_marker(const ParticleFilter &tracker)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "local_enu";
    marker.header.stamp = ros::Time::now();
    marker.ns = "global_id_text";
    marker.id = tracker.get_id();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    auto mean_particle = tracker.get_mean_particle();
    marker.pose.position.x = mean_particle.get_x();
    marker.pose.position.y = mean_particle.get_y();
    marker.pose.position.z = 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 2.0 * marker_scale;
    marker.color = get_color(tracker.get_id());
    marker.color.a = 1.0;
    marker.text = class_id_to_label.at(tracker.get_class_id()) + std::to_string(tracker.get_id());
    return marker;
}

std::vector<visualization_msgs::Marker> make_particle_probability_text_markers(const ParticleFilter &tracker)
{
    std::vector<visualization_msgs::Marker> markers;
    int particle_counter = 0;
    for (auto &p : tracker.get_particles())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "local_enu";
        marker.header.stamp = ros::Time::now();
        marker.ns = "particle_probability_text";
        marker.id = tracker.get_id() * 1000 + particle_counter;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = p.get_x();
        marker.pose.position.y = p.get_y();
        marker.pose.position.z = 1.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.5 * marker_scale;
        marker.color = get_color(tracker.get_id());
        marker.color.a = 0.8;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(5) << p.get_probability();
        marker.text = ss.str();
        markers.push_back(marker);
        particle_counter += 1;
    }
    return markers;
}

void particle_filters_belief_callback(ipp_belief::ParticleFiltersBelief msg)
{
    ma.markers.clear();
    auto belief_manager = tracking::belief_manager_from_ros_msg(msg);
    for (auto &[id, tracker] : belief_manager.id_to_trackers)
    {
        double alpha = std::clamp(1.0 / tracker.get_num_particles() * marker_alpha_multiplier, 0.0, 1.0);
        ParticleFilterVisualizer visualizer(get_color(id), marker_scale, alpha);
        visualizer.update(tracker);
        ma.markers.push_back(visualizer.get_marker());
        ma.markers.push_back(make_global_id_text_marker(tracker));
        auto particle_prob_text_markers = make_particle_probability_text_markers(tracker);
        ma.markers.insert(ma.markers.end(), particle_prob_text_markers.begin(), particle_prob_text_markers.end());
    }
    particle_pub.publish(ma);
}

std::unordered_map<unsigned int, std::string> parse_class_id_to_label_map(XmlRpc::XmlRpcValue class_controls)
{
    std::unordered_map<unsigned int, std::string> class_id_to_label;
    for (int i = 0; i < class_controls.size(); i++)
    {
        XmlRpc::XmlRpcValue class_control = class_controls[i];
        int class_id = class_control["class_id"];
        std::string class_label = class_control["class_label"];
        ROS_INFO_STREAM("class_id: " << class_id << " <--> class_label: " << class_label);
        class_id_to_label[class_id] = class_label;
    }
    return class_id_to_label;
}

int main(int argc, char **argv)
{
    srand(2204);
    ros::init(argc, argv, "ipp_belief_visualizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    class_id_to_label = parse_class_id_to_label_map(ros_utils::get_param<XmlRpc::XmlRpcValue>(pnh, "class_controls"));

    std::string input_topic = ros_utils::get_param<std::string>(nh, "belief_output_topic");
    ros::Subscriber particle_message_sub = nh.subscribe(input_topic, 10, particle_filters_belief_callback);
    particle_pub = nh.advertise<visualization_msgs::MarkerArray>("particle_filters_visualizer", 0);

    marker_scale = ros_utils::get_param<double>(pnh, "marker_scale");
    marker_alpha_multiplier = ros_utils::get_param<double>(pnh, "marker_alpha_multiplier");

    ros::spin();
}