#pragma once

#include <tuple>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/MarkerArray.h>

#include "planner_map_interfaces/ros_utils.h"

#include <planner_map_interfaces/Plan.h>
#include <planner_map_interfaces/PlanRequest.h>
#include <planner_map_interfaces/Waypoint.h>

#include "ipp_belief/observation.h"
#include "ipp_belief/state.h"
#include "ipp_planners/Planner.h"
#include "ipp_planners/Tigris.h"
#ifdef USE_MCTS
#include "ipp_planners/MCTS.h"
#endif
#include "ipp_planners/GreedySearchPlanner.h"
#include "ipp_planners/GreedyTrackPlanner.h"
#include "ipp_planners/RandomSearchPlanner.h"
#include "ipp_planners/RandomTrackPlanner.h"
#include "ipp_planners/Tracking.h"
#include "ipp_planners/CoveragePlanner.h"
#include "ipp_planners/PrimTree.h"
#include "ipp_planners/PrimTreeBnB.h"
#include "ipp_planners/MCTSSearch.h"

using namespace ipp;

double visualization_scale = 1.0;

/**
 * @brief
 *
 * @param tree_nodes assumes information is between -1, 1
 * @param local_frame
 * @return visualization_msgs::MarkerArray
 */
template <class NodeT>
visualization_msgs::MarkerArray visualize_node_info_gain_text(std::vector<NodeT *> tree_nodes, std::string local_frame)
{
    double min_information = std::numeric_limits<double>::max();
    double max_information = std::numeric_limits<double>::lowest();
    for (auto node : tree_nodes)
    {
        if (node->information < min_information)
        {
            min_information = node->information;
        }
        if (node->information > max_information)
        {
            max_information = node->information;
        }
    }
    if (min_information == max_information)
    {
        max_information = min_information + 1;
    }

    visualization_msgs::MarkerArray text_marker_array;
    // apply color based on information
    int text_id = 0;
    for (auto node : tree_nodes)
    {
        auto *s_ = node->state->template as<XYZPsiStateSpace::StateType>();
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = local_frame;
        text_marker.header.stamp = ros::Time();
        text_marker.ns = "node_information";
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.id = text_id++;
        text_marker.scale.z = 5 * visualization_scale;

        geometry_msgs::Point new_point;

        new_point.x = text_marker.pose.position.x = s_->getX();
        new_point.y = text_marker.pose.position.y = s_->getY();
        new_point.z = text_marker.pose.position.z = s_->getZ();

        text_marker.pose.position.z += 10;

        std_msgs::ColorRGBA new_color;
        double normalized_information = (node->information - min_information) / (max_information - min_information);

        // text_marker.scale.z = 30 * normalized_information + 10.0; // at least size 10
        if (normalized_information == 1.0)
        { // make the top ones bright green
            new_color.g = 1.0;
            new_color.r = 0.0;
            new_color.b = 0.0;
        }
        else
        {
            new_color.r = text_marker.color.r = normalized_information;
            new_color.b = text_marker.color.b = (1 - normalized_information);
        }
        new_color.a = text_marker.color.a = 1.0;

        std::stringstream stream;
        stream << "d=" << node->depth << ": " << /*std::fixed << std::setprecision(10) <<*/ node->information;
        text_marker.text = stream.str();
        text_marker_array.markers.push_back(text_marker);
    }
    return text_marker_array;
}



#ifdef USE_MCTS
visualization_msgs::MarkerArray visualize_node_avg_value_text(std::vector<MCTSNode *> tree_nodes, std::string local_frame)
{
    auto value_fn = [](MCTSNode *node)
    {
        double total_budget = node->cost.value() + node->budget_remaining;
        return node->get_value() / (total_budget / 50);
    };
    double min_information = std::numeric_limits<double>::max();
    double max_information = std::numeric_limits<double>::lowest();
    for (auto node : tree_nodes)
    {
        if (node->name == "R") // root gets too negative, ignore it for coloring
        {
            continue;
        }
        if (value_fn(node) < min_information)
        {
            min_information = value_fn(node);
        }
        if (value_fn(node) > max_information)
        {
            max_information = value_fn(node);
        }
    }
    if (min_information == max_information)
    {
        max_information = min_information + 1;
    }

    visualization_msgs::MarkerArray text_marker_array;
    // apply color based on information
    int text_id = 0;
    for (auto node : tree_nodes)
    {
        auto *s_ = node->state->as<XYZPsiStateSpace::StateType>();
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = local_frame;
        text_marker.header.stamp = ros::Time();
        text_marker.ns = "avg_value";
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.id = text_id++;
        text_marker.scale.z = 5 * visualization_scale;

        geometry_msgs::Point new_point;

        new_point.x = text_marker.pose.position.x = s_->getX();
        new_point.y = text_marker.pose.position.y = s_->getY();
        new_point.z = text_marker.pose.position.z = s_->getZ();

        text_marker.pose.position.z += 10;

        std_msgs::ColorRGBA new_color;
        double normalized_information = (value_fn(node) - min_information) / (max_information - min_information);
        // ROS_DEBUG_STREAM("normalized_information: " << normalized_information);
        new_color.g = normalized_information;
        // new_color.r = 0.5;
        new_color.b = 1 - normalized_information;
        new_color.a = 0.9 * std::pow(normalized_information, 2) + 0.1;
        // text_marker.scale.z = 30 * normalized_information + 10.0; // at least size 10
        text_marker.color = new_color;
        //
        std::stringstream stream;
        stream << /*node->name << ":" << std::fixed << std::setprecision(10) <<*/ value_fn(node);
        text_marker.text = stream.str();
        text_marker_array.markers.push_back(text_marker);
    }
    return text_marker_array;
}
#endif

/**
 * @brief
 *
 * @param xyzi_samples vector of sampled points with information values
 * @param local_frame
 * @return std::tuple<visualization_msgs::Marker, visualization_msgs::MarkerArray> ; first marker is the sampled points, second is the information in text
 */
std::tuple<visualization_msgs::Marker, visualization_msgs::MarkerArray> visualize_sampled_points(const std::vector<std::vector<double>> &xyzi_samples, std::string local_frame)
{
    // ROS_DEBUG_STREAM("Visualizing " << xyzi_samples.size() << " sampled points");
    double min_information = std::numeric_limits<double>::max();
    double max_information = std::numeric_limits<double>::lowest();
    for (auto &xyzi : xyzi_samples)
    {
        if (xyzi[3] < min_information)
        {
            min_information = xyzi[3];
        }
        if (xyzi[3] > max_information)
        {
            max_information = xyzi[3];
        }
    }
    // ROS_WARN("Visualizing sampled points, min_information: %f is the same as max_information: %f", min_information, max_information);
    if (min_information == max_information)
    {
        max_information = min_information + 1;
    }

    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = local_frame;
    points_marker.header.stamp = ros::Time();
    points_marker.ns = "samples";
    // points_marker.frame_locked = true;
    points_marker.id = 0;
    points_marker.type = visualization_msgs::Marker::CUBE_LIST;
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.position.x = 0.0;
    points_marker.pose.position.y = 0.0;
    points_marker.pose.position.z = 0.0;
    points_marker.pose.orientation.x = 0.0;
    points_marker.pose.orientation.y = 0.0;
    points_marker.pose.orientation.z = 0.0;
    points_marker.pose.orientation.w = 1.0;
    points_marker.scale.x = .1 * visualization_scale;
    points_marker.scale.y = .1 * visualization_scale;
    points_marker.scale.z = .1 * visualization_scale;
    points_marker.color.a = 1.0;
    points_marker.color.r = 1.0;
    points_marker.color.g = 1.0;
    points_marker.color.b = 1.0;
    double color_scaler = 0.0;

    visualization_msgs::MarkerArray text_marker_array;
    // apply color based on information
    int text_id = 0;
    for (auto &xyzi : xyzi_samples)
    {
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = local_frame;
        text_marker.header.stamp = points_marker.header.stamp;
        text_marker.ns = "sample_information";
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.id = text_id++;
        text_marker.scale.z = 0.1 * visualization_scale;

        geometry_msgs::Point new_point;

        new_point.x = text_marker.pose.position.x = xyzi[0];
        new_point.y = text_marker.pose.position.y = xyzi[1];
        new_point.z = text_marker.pose.position.z = xyzi[2];
        points_marker.points.push_back(new_point);

        text_marker.pose.position.z += 10;

        std_msgs::ColorRGBA new_color;
        double normalized_information = (xyzi[3] - min_information) / (max_information - min_information);
        new_color.r = text_marker.color.r = normalized_information;
        new_color.b = text_marker.color.b = (1 - normalized_information);
        new_color.a = text_marker.color.a = 1.0;
        points_marker.colors.push_back(new_color);

        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << xyzi[3];
        text_marker.text = stream.str();
        text_marker_array.markers.push_back(text_marker);
    }
    return std::make_tuple(points_marker, text_marker_array);
}

/**
 * @brief generic function for visualizing list of points
 *
 * @param xyzi_samples vector of points
 * @param local_frame
 * @return visualization_msgs::Marker
 */
visualization_msgs::MarkerArray visualize_points(const std::vector<std::vector<double>> &xyzi_points, std::string local_frame, std::vector<double> desired_color)
{
    // ROS_INFO("Visualizing points as arrows");
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "nodes";
    // m.frame_locked = true;
    m.id = 0;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.3 * visualization_scale;
    m.scale.y = .1 * visualization_scale;
    m.scale.z = .1 * visualization_scale;
    m.color.a = desired_color[0];
    m.color.r = desired_color[1];
    m.color.g = desired_color[2];
    m.color.b = desired_color[3];

    int marker_id = 0;
    tf2::Quaternion myQuaternion;
    for (auto &xyzi : xyzi_points)
    {
        m.id = marker_id++;
        m.pose.position.x = xyzi[0];
        m.pose.position.y = xyzi[1];
        m.pose.position.z = xyzi[2];
        myQuaternion.setRPY(0, 0, xyzi[3]);
        m.pose.orientation.x = myQuaternion[0];
        m.pose.orientation.y = myQuaternion[1];
        m.pose.orientation.z = myQuaternion[2];
        m.pose.orientation.w = myQuaternion[3];
        ma.markers.push_back(m);
    }
    return ma;
}

/**
 * @brief Visualize the final path of the robot
 *
 * @param planned_path_vector
 * @param space_ptr
 * @param local_frame
 * @param dubins_vel
 * @param max_kappa
 * @return visualization_msgs::Marker
 */
visualization_msgs::Marker visualize_final_path(og::PathGeometric planned_path,
                                                ompl::base::SpaceInformationPtr space_ptr,
                                                std::string local_frame,
                                                double dubins_vel,
                                                double max_kappa)
{

    // ROS_INFO("Displaying Final Path");
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
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    geometry_msgs::Pose new_pose;

    ob::State *state;
    auto *node_pt = state->as<XYZPsiStateSpace::StateType>();
    auto &path_states = planned_path.getStates();
    if (path_states.size() > 0)
    {
        for (auto it = path_states.begin(); it != path_states.end() - 1; ++it)
        {
            auto s1 = *it;
            auto s2 = *(it + 1);

            geometry_msgs::Point child_point;
            node_pt = s1->as<XYZPsiStateSpace::StateType>();
            child_point.x = node_pt->getX();
            child_point.y = node_pt->getY();
            child_point.z = node_pt->getZ();

            geometry_msgs::Point parent_point;
            node_pt = s2->as<XYZPsiStateSpace::StateType>();
            parent_point.x = node_pt->getX();
            parent_point.y = node_pt->getY();
            parent_point.z = node_pt->getZ();

            m.points.push_back(child_point);
            m.points.push_back(parent_point);
        }
        // std::cout << "Number of Segments in best path: " << counter << std::endl;
    }
    return m;
}

visualization_msgs::Marker visualize_dense_path(planner_map_interfaces::Plan dense_path,
                                                std::string local_frame)
{

    // ROS_INFO("Displaying Final Path");
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "nodes";
    // m.frame_locked = true;
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.1 * visualization_scale;
    m.color.a = 0.7;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    for (int i = 0; i < dense_path.plan.size(); i++)
    {
        geometry_msgs::Point parent_point = dense_path.plan[i].position.position;
        // geometry_msgs::Point child_point = dense_path.plan[i+1].position.position;

        m.points.push_back(parent_point);
        // m.points.push_back(child_point);
    }
    // std::cout << "Number of Segments in best path: " << counter << std::endl;
    return m;
}

visualization_msgs::Marker visualize_map(const SearchMap &map_repr, bool vis_path_belief, std::string local_frame, double max_alpha = 0.4)
{

    // ROS_INFO("Displaying Belief Map");
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "nodes";
    // m.frame_locked = true;
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = -2.0; // my height is 10. particles are at -0.5, so put this at -12.0
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = map_repr.map_resolution;
    m.scale.y = map_repr.map_resolution;
    double offset = map_repr.map_resolution / 2;
    m.scale.z = .01 * visualization_scale;
    m.color.a = max_alpha;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    double color_scaler = 0.0;

    for (int i = 0; i < map_repr.num_rows; i++)
    {
        for (int j = 0; j < map_repr.num_cols; j++)
        {
            geometry_msgs::Point new_point;
            new_point.x = i * map_repr.map_resolution + map_repr.x_start + offset;
            new_point.y = j * map_repr.map_resolution + map_repr.y_start + offset;
            new_point.z = -1;

            std_msgs::ColorRGBA new_color;

            if (vis_path_belief)
                color_scaler = map_repr.copy_map.at(i).at(j);
            else
                color_scaler = map_repr.map.at(i).at(j);
            new_color.r = (color_scaler < 1) ? color_scaler : 1;
            new_color.g = 0;
            new_color.b = (color_scaler < .8) ? .8 - color_scaler * .8 : 0;
            new_color.a = (color_scaler == 0) ? 0 : max_alpha;

            m.colors.push_back(new_color);
            m.points.push_back(new_point);
        }
    }
    return m;
}

visualization_msgs::Marker visualize_tree_line(const std::vector<TopoNode *> &nodes,
                                               std::string local_frame)
{
    ROS_INFO_STREAM("publishing tree");
    // ROS_INFO("Displaying Tree with Dubins paths");
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "nodes";
    // m.frame_locked = true;
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    // m.lifetime = ros::Duration(1);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = .1 * visualization_scale;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0; // TODO change color gradient
    geometry_msgs::Pose new_pose;

    TopoNode *node_ptr = nodes[nodes.size() - 1];
    auto *node_pt = node_ptr->state->template as<XYZPsiStateSpace::StateType>();

    for (auto *node : nodes)
    {
        if (node->parent == nullptr)
        {
            continue; // skip the root, the root has no parent edge to draw
        }

        std::vector<double> node_state = node->get_node_start_pose();
        std::vector<double> parent_state = node->parent_topo->get_node_start_pose();

        geometry_msgs::Point child_point;
        child_point.x = node_state[0];
        child_point.y = node_state[1];
        child_point.z = node_state[2];
        geometry_msgs::Point parent_point;
        parent_point.x = parent_state[0];
        parent_point.y = parent_state[1];
        parent_point.z = parent_state[2];

        m.points.push_back(child_point);
        m.points.push_back(parent_point);
    }
    return m;
}
/**
 * @brief Visualize the tree while considering the dubins path
 *
 * @param tree
 * @param space_ptr
 * @param local_frame
 * @return visualization_msgs::Marker
 */
template <class NodeT>
visualization_msgs::Marker visualize_tree_dubins(const std::vector<NodeT *> &nodes,
                                                 ompl::base::SpaceInformationPtr space_ptr,
                                                 std::string local_frame)
{

    // TODO: hacky function to use value in case of MCTS, information in case of TIGRIS. need to rethink this and cleanup
    auto value_fn = [](NodeT *node)
    {
#ifdef USE_MCTS
        MCTSNode *mcts_node = dynamic_cast<MCTSNode *>(node);
        if (mcts_node)
        {
            double total_budget = mcts_node->cost.value() + mcts_node->budget_remaining;
            return mcts_node->get_value() / (total_budget / 50);
        }
        else
        {
            return node->information;
        }
#else
        return node->information;
#endif
    };
    double min_information = std::numeric_limits<double>::max();
    double max_information = std::numeric_limits<double>::lowest();
    for (auto node : nodes)
    {
        if (value_fn(node) < min_information)
        {
            min_information = value_fn(node);
        }
        if (value_fn(node) > max_information)
        {
            max_information = value_fn(node);
        }
    }
    if (min_information == max_information)
    {
        max_information = min_information + 1;
    }

    // ROS_INFO("Displaying Tree with Dubins paths");
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "nodes";
    // m.frame_locked = true;
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    // m.lifetime = ros::Duration(1);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = .1 * visualization_scale;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0; // TODO change color gradient
    geometry_msgs::Pose new_pose;

    NodeT *node_ptr = nodes[nodes.size() - 1];
    auto *node_pt = node_ptr->state->template as<XYZPsiStateSpace::StateType>();

    for (auto *node : nodes)
    {
        if (node->parent == nullptr)
            continue; // skip the root, the root has no parent edge to draw
        auto &path_states = node->edge_trochoid.getStates();
        for (auto it = path_states.begin(); it < path_states.end() - 2; it += 2)
        {
            auto s1 = *it;
            auto s2 = *(it + 1);

            geometry_msgs::Point child_point;
            node_pt = s1->template as<XYZPsiStateSpace::StateType>();
            child_point.x = node_pt->getX();
            child_point.y = node_pt->getY();
            child_point.z = node_pt->getZ();

            geometry_msgs::Point parent_point;
            node_pt = s2->template as<XYZPsiStateSpace::StateType>();
            parent_point.x = node_pt->getX();
            parent_point.y = node_pt->getY();
            parent_point.z = node_pt->getZ();

            std_msgs::ColorRGBA new_color;
            // new_color.r = (color_scaler < .9) ? color_scaler*.9 + .1 : 1;
            // new_color.g = (color_scaler < .5) ? 1 - color_scaler*.5 : .5;
            // new_color.b = (color_scaler < .5) ? color_scaler*.5 + .5 : 1;
            double normalized_information = (value_fn(node) - min_information) / (max_information - min_information);

            // make the tree gray to bright green
            // new_color.g = std::sqrt(normalized_information) * 0.4 + 0.6;
            // new_color.r =  0.6;
            // new_color.b =  1 - new_color.g + 0.6;
            new_color.g = normalized_information * 0.5 + 0.5;
            new_color.r = 0.5;
            new_color.b = (1 - normalized_information) * 0.5 + 0.5;
            m.colors.push_back(new_color);
            m.colors.push_back(new_color);
            m.points.push_back(child_point);
            m.points.push_back(parent_point);
        }
    }
    return m;
}

template <class NodeT>
visualization_msgs::MarkerArray visualize_nodes_arrow(const std::vector<NodeT *> &nodes, std::string local_frame)
{
    // ROS_INFO("Visualizing nodes with arrows");
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "nodes";
    // m.frame_locked = true;
    m.id = 0;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.1 * visualization_scale;
    m.scale.y = .05 * visualization_scale;
    m.scale.z = .05 * visualization_scale;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;

    tf2::Quaternion myQuaternion;
    for (size_t i = 0; i < nodes.size(); i++)
    {
        m.id = i;
        auto *node_pt = nodes[i]->state->template as<XYZPsiStateSpace::StateType>();
        m.pose.position.x = node_pt->getX();
        m.pose.position.y = node_pt->getY();
        m.pose.position.z = node_pt->getZ();
        myQuaternion.setRPY(0, 0, node_pt->getPsi());
        m.pose.orientation.x = myQuaternion[0];
        m.pose.orientation.y = myQuaternion[1];
        m.pose.orientation.z = myQuaternion[2];
        m.pose.orientation.w = myQuaternion[3];

        ma.markers.push_back(m);
    }
    return ma;
}

visualization_msgs::MarkerArray visualize_counter_detect(std::string local_frame,
                                                         double counter_detect_radius,
                                                         std::vector<std::vector<double>> &target_prior)
{
    // ROS_INFO("Visualizing counterdetection cylinders");
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "nodes";
    // m.frame_locked = true;
    m.id = 0;
    m.type = visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = counter_detect_radius * 2;
    m.scale.y = counter_detect_radius * 2;
    m.scale.z = 20 * visualization_scale;
    m.color.a = 0.3;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;

    for (size_t i = 0; i < target_prior.size(); i++)
    {
        m.id = i;
        m.pose.position.x = target_prior[i][0];
        m.pose.position.y = target_prior[i][1];
        m.pose.position.z = m.scale.z / 2;

        ma.markers.push_back(m);
    }
    return ma;
}

visualization_msgs::Marker visualize_counter_detect_2d(size_t i,
                                                       std::string local_frame,
                                                       double counter_detect_radius,
                                                       std::vector<std::vector<double>> &target_prior)
{
    /*
    for(size_t i=0; i<target_prior.size(); i++){
        for (int j=0; j<= 360; j += 20)
        {
            geometry_msgs::Point point;
            // m.id = i*(j + 1);
            double x = counter_detect_radius * cos(j * M_PI / 180.0) + target_prior[i][0];  // target_prior[i][0]
            double y = counter_detect_radius * sin(j * M_PI / 180.0) + target_prior[i][1];  // target_prior[i][1]
            point.x = x;
            point.y = y;
            // std::cout << "HI\n";
            point.z = 100;
            m.points.push_back(point);
        }
    }
    */
    // ROS_INFO("Visualizing nodes as spheres");
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "counter_cir";
    // m.frame_locked = true;
    m.id = i;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 6.5 * visualization_scale;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;

    for (int j = 0; j <= 360; j += 20)
    {
        geometry_msgs::Point point;
        double x = counter_detect_radius * cos(j * M_PI / 180.0) + target_prior[i][0]; // target_prior[i][0]
        double y = counter_detect_radius * sin(j * M_PI / 180.0) + target_prior[i][1]; // target_prior[i][1]
        point.x = x;
        point.y = y;
        point.z = 100;
        m.points.push_back(point);
    }

    return m;
}

visualization_msgs::Marker visualize_nodes_sphere(
    const std::vector<TreeNode *> &nodes, std::string local_frame)
{
    // ROS_INFO("Visualizing nodes as spheres");
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "nodes";
    // m.frame_locked = true;
    m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.15 * visualization_scale;
    m.color.a = 1.0;
    m.color.r = 8.0;
    m.color.g = 5.0;
    m.color.b = 0.0;

    for (size_t i = 0; i < nodes.size(); i++)
    {
        geometry_msgs::Point new_point;
        auto *node_pt = nodes[i]->state->as<XYZPsiStateSpace::StateType>();
        new_point.x = node_pt->getX();
        new_point.y = node_pt->getY();
        new_point.z = node_pt->getZ();

        m.points.push_back(new_point);
    }

    return m;
}

visualization_msgs::Marker visualize_final_path_views(std::vector<XYZPsiStateSpace::StateType *> &planned_path,
                                                      SensorParams sensor_params,
                                                      std::string local_frame)
{
    // ROS_INFO("Visualizing final path frustrum");
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
    m.scale.x = .2 * visualization_scale;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    for (int i = 0; i < planned_path.size(); i++)
    {
        auto *node_pt = planned_path[i];

        std::vector<std::vector<double>> q_rotated = rotated_camera_fov(sensor_params, /*roll*/ 0.0, /*pitch*/ sensor_params.pitch, /*yaw*/ node_pt->getPsi());
        std::vector<double> node_pos = {node_pt->getX(), node_pt->getY(), node_pt->getZ()};
        std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pos, q_rotated, sensor_params.highest_max_range);

        for (int i = 0; i < projected_camera_bounds.size(); i++)
        {
            geometry_msgs::Point new_point;
            new_point.x = projected_camera_bounds[i][0];
            new_point.y = projected_camera_bounds[i][1];
            new_point.z = projected_camera_bounds[i][2];

            geometry_msgs::Point newer_point;
            newer_point.x = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][0];
            newer_point.y = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][1];
            newer_point.z = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][2];

            geometry_msgs::Point cam_point;
            cam_point.x = node_pt->getX();
            cam_point.y = node_pt->getY();
            cam_point.z = node_pt->getZ();

            geometry_msgs::Point new_point_copy;
            new_point_copy.x = projected_camera_bounds[i][0];
            new_point_copy.y = projected_camera_bounds[i][1];
            new_point_copy.z = projected_camera_bounds[i][2];

            m.points.push_back(new_point);
            m.points.push_back(newer_point);
            m.points.push_back(new_point_copy);
            m.points.push_back(cam_point);
        }
    }
    return m;
}

visualization_msgs::Marker visualize_final_path_views(const ompl::NearestNeighbors<TreeNode *> &tree,
                                                      TreeNode *node_ptr,
                                                      SensorParams sensor_params,
                                                      std::string local_frame)
{
    // ROS_INFO("Visualizing final path frustrum");
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
    m.scale.x = .1 * visualization_scale;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    while (node_ptr != nullptr)
    {
        auto *node_pt = node_ptr->state->as<XYZPsiStateSpace::StateType>();

        std::vector<std::vector<double>> q_rotated = rotated_camera_fov(sensor_params, /*roll*/ 0.0, /*pitch*/ sensor_params.pitch, /*yaw*/ node_pt->getPsi());
        std::vector<double> node_pos = {node_pt->getX(), node_pt->getY(), node_pt->getZ()};
        std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pos, q_rotated, sensor_params.highest_max_range);

        for (int i = 0; i < projected_camera_bounds.size(); i++)
        {
            geometry_msgs::Point new_point;
            new_point.x = projected_camera_bounds[i][0];
            new_point.y = projected_camera_bounds[i][1];
            new_point.z = projected_camera_bounds[i][2];

            geometry_msgs::Point newer_point;
            newer_point.x = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][0];
            newer_point.y = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][1];
            newer_point.z = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][2];

            geometry_msgs::Point cam_point;
            cam_point.x = node_pt->getX();
            cam_point.y = node_pt->getY();
            cam_point.z = node_pt->getZ();

            geometry_msgs::Point new_point_copy;
            new_point_copy.x = projected_camera_bounds[i][0];
            new_point_copy.y = projected_camera_bounds[i][1];
            new_point_copy.z = projected_camera_bounds[i][2];

            m.points.push_back(new_point);
            m.points.push_back(newer_point);
            m.points.push_back(new_point_copy);
            m.points.push_back(cam_point);
        }

        node_ptr = node_ptr->parent;
    }
    return m;
}

visualization_msgs::Marker visualize_final_path_observations(TreeNode *node_ptr,
                                                             double desired_speed,
                                                             SensorParams sensor_params,
                                                             double observation_discretization_distance,
                                                             std::string local_frame)
{
    auto [observations, time_deltas] = ipp::discretize_observations(*node_ptr, desired_speed, sensor_params, std::numeric_limits<int>::max(), observation_discretization_distance);
    // ROS_INFO("Visualizing final path frustrum");
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
    m.scale.x = .1 * visualization_scale;
    m.color.a = 0.5;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    // ROS_INFO("Observations size: %d", observations.size());
    for (auto &obs : observations)
    {
        auto d = obs.get_vantage_point();
        std::vector<std::vector<double>> q_rotated = rotated_camera_fov(sensor_params, /*roll*/ 0.0, /*pitch*/ sensor_params.pitch, /*yaw*/ d.get_heading());
        std::vector<double> node_pos = {d.get_x(), d.get_y(), d.get_z()};
        std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pos, q_rotated, sensor_params.highest_max_range);

        for (int i = 0; i < projected_camera_bounds.size(); i++)
        {
            geometry_msgs::Point new_point;
            new_point.x = projected_camera_bounds[i][0];
            new_point.y = projected_camera_bounds[i][1];
            new_point.z = projected_camera_bounds[i][2];

            geometry_msgs::Point newer_point;
            newer_point.x = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][0];
            newer_point.y = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][1];
            newer_point.z = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][2];

            geometry_msgs::Point cam_point;
            cam_point.x = d.get_x();
            cam_point.y = d.get_y();
            cam_point.z = d.get_z();

            geometry_msgs::Point new_point_copy;
            new_point_copy.x = projected_camera_bounds[i][0];
            new_point_copy.y = projected_camera_bounds[i][1];
            new_point_copy.z = projected_camera_bounds[i][2];

            m.points.push_back(new_point);
            m.points.push_back(newer_point);
            m.points.push_back(new_point_copy);
            m.points.push_back(cam_point);
        }
    }
    return m;
}

visualization_msgs::Marker visualize_frustum(double &x, double &y, double &z, double &yaw, double &pitch, double &roll,
                                             SensorParams sensor_params,
                                             std::string local_frame,
                                             double marker_scale,
                                             double alpha = 1.0)
{

    // ROS_INFO("Visualizing final path frustrum");
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
    m.scale.x = 0.025 * marker_scale;
    m.color.a = alpha;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;

    std::vector<std::vector<double>> q_rotated = rotated_camera_fov(sensor_params, /*roll*/ roll, /*pitch*/ pitch, /*yaw*/ yaw);
    std::vector<double> node_pos = {x, y, z};
    std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pos, q_rotated, sensor_params.highest_max_range);

    // for debug purpose, it shows the entire frustum with max sensor range, check if intersect with the ground plane properly

    // std::vector<std::vector<double>> raw_frustum = project_raw_frustum(node_pos, q_rotated, sensor_params.highest_max_range);
    // for (size_t j = 0; j < raw_frustum.size(); j++)
    // {
    //     geometry_msgs::Point new_end;
    //     new_end.x = raw_frustum[j][0];
    //     new_end.y = raw_frustum[j][1];
    //     new_end.z = raw_frustum[j][2];

    //     geometry_msgs::Point newer_end;
    //     newer_end.x = raw_frustum[(int)((j + 1) % raw_frustum.size())][0];
    //     newer_end.y = raw_frustum[(int)((j + 1) % raw_frustum.size())][1];
    //     newer_end.z = raw_frustum[(int)((j + 1) % raw_frustum.size())][2];

    //     geometry_msgs::Point odom_point;
    //     odom_point.x = x;
    //     odom_point.y = y;
    //     odom_point.z = z;

    //     geometry_msgs::Point new_end_copy;
    //     new_end_copy.x = raw_frustum[j][0];
    //     new_end_copy.y = raw_frustum[j][1];
    //     new_end_copy.z = raw_frustum[j][2];

    //     m.points.push_back(new_end);
    //     m.points.push_back(newer_end);
    //     m.points.push_back(new_end_copy);
    //     m.points.push_back(odom_point);
    // }

    for (int i = 0; i < projected_camera_bounds.size(); i++)
    {
        geometry_msgs::Point new_point;
        new_point.x = projected_camera_bounds[i][0];
        new_point.y = projected_camera_bounds[i][1];
        new_point.z = projected_camera_bounds[i][2];

        geometry_msgs::Point newer_point;
        newer_point.x = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][0];
        newer_point.y = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][1];
        newer_point.z = projected_camera_bounds[(int)((i + 1) % projected_camera_bounds.size())][2];

        geometry_msgs::Point cam_point;
        cam_point.x = x;
        cam_point.y = y;
        cam_point.z = z;

        geometry_msgs::Point new_point_copy;
        new_point_copy.x = projected_camera_bounds[i][0];
        new_point_copy.y = projected_camera_bounds[i][1];
        new_point_copy.z = projected_camera_bounds[i][2];

        m.points.push_back(new_point);
        m.points.push_back(newer_point);
        m.points.push_back(new_point_copy);
        m.points.push_back(cam_point);
    }

    return m;
}

visualization_msgs::Marker visualize_planner_boundary(geometry_msgs::Polygon bounds,
                                                      std::string local_frame)
{
    // ROS_INFO("Displaying Final Path");
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "nodes";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.2 * visualization_scale;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    geometry_msgs::Pose new_pose;

    for (auto point32 : bounds.points)
    {
        geometry_msgs::Point point;
        point.x = point32.x;
        point.y = point32.y;
        point.z = point32.z;
        m.points.push_back(point);
    }
    if (bounds.points.size() > 0)
    {
        geometry_msgs::Point point;
        point.x = bounds.points[0].x;
        point.y = bounds.points[0].y;
        point.z = bounds.points[0].z;
        m.points.push_back(point);
    }
    return m;
}

visualization_msgs::Marker visualize_keep_out_2d(std::string local_frame,
                                                 std::vector<KeepOutZone> &keep_out_zones)
{
    // ROS_INFO("Visualizing nodes as spheres");
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "counter_cir";
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
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    for (int k = 0; k < keep_out_zones.size(); k++)
    {
        for (int i = 0; i < 360; i += 10)
        {
            geometry_msgs::Point new_point;
            new_point.x = keep_out_zones[k].radius * cos(i * M_PI / 180.0) + keep_out_zones[k].center.first;
            new_point.y = keep_out_zones[k].radius * sin(i * M_PI / 180.0) + keep_out_zones[k].center.second;
            new_point.z = 1.0;
            m.points.push_back(new_point);
        }
    }
    // for (int j = 0; j <= 360; j += 20)
    // {
    //     geometry_msgs::Point point;
    //     double x = counter_detect_radius * cos(j * M_PI / 180.0) + target_prior[i][0]; // target_prior[i][0]
    //     double y = counter_detect_radius * sin(j * M_PI / 180.0) + target_prior[i][1]; // target_prior[i][1]
    //     point.x = x;
    //     point.y = y;
    //     point.z = 100;
    //     m.points.push_back(point);
    // }

    return m;
}

visualization_msgs::Marker visualize_setpoints(std::string local_frame, 
                                               std::vector<double> pose_to_plan_from,
                                               planner_map_interfaces::Plan dense_path,
                                               std::vector<int> waypoint_map)
{
    visualization_msgs::Marker m;
    m.header.frame_id = local_frame;
    m.header.stamp = ros::Time();
    m.ns = "waypoint_vis";
    m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.25 * visualization_scale;
    m.scale.y = 0.25 * visualization_scale;
    m.scale.z = 0.25 * visualization_scale;
    m.color.a = 1.0;
    //iterate through waypoint map and dense path to determine where waypoints exist
    int prev_waypoint = 0.0;
    for(int idx = 0; idx < waypoint_map.size(); ++idx)
    {
        int waypoint = waypoint_map[idx];
        geometry_msgs::Point parent_point = dense_path.plan[idx].position.position;
        std_msgs::ColorRGBA setpoint_c;
        setpoint_c.a = 0.5;
        setpoint_c.r = 0.0;
        setpoint_c.g = 1.0;
        setpoint_c.b = 0.0;
        if(waypoint > prev_waypoint)
        {
            m.points.push_back(parent_point);
            m.colors.push_back(setpoint_c);
            prev_waypoint = waypoint;
        }
    }
    // append start pose last so it shows up on top of previous markers
    geometry_msgs::Point start_point;
    start_point.x = pose_to_plan_from[0];
    start_point.y = pose_to_plan_from[1];
    start_point.z = pose_to_plan_from[2]+1;
    m.points.push_back(start_point);
    std_msgs::ColorRGBA start_c;
    start_c.a = 1.0;    
    start_c.r = 1.0;
    start_c.g = 0.0;
    start_c.b = 0.0;
    m.colors.push_back(start_c);
    return m;
}

visualization_msgs::MarkerArray visualize_waypoint_nums(std::string local_frame,
                                                       planner_map_interfaces::Plan dense_path,
                                                       std::vector<int> waypoint_map)
{
    visualization_msgs::MarkerArray text_marker_array;
    //iterate through dense path & waypoint map to determine where waypoints are
    int prev_waypoint = 0.0;
    for(int idx = 0; idx < waypoint_map.size(); ++idx)
    {
        int waypoint = waypoint_map[idx];
        if(waypoint > prev_waypoint)
        {
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = local_frame;
            text_marker.header.stamp = ros::Time();
            text_marker.ns = "waypoint_num";
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.id = waypoint;
            text_marker.scale.z = 0.75 * visualization_scale;
            
            geometry_msgs::Point new_point;
            new_point.x = text_marker.pose.position.x = dense_path.plan[idx].position.position.x;
            new_point.y = text_marker.pose.position.y = dense_path.plan[idx].position.position.y;
            new_point.z = text_marker.pose.position.z = dense_path.plan[idx].position.position.z;
            text_marker.pose.position.z += 10;

            text_marker.color.a = 1.0;
            text_marker.color.r = 0.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 0.0;

            std::stringstream stream;
            stream << std::to_string(waypoint);
            text_marker.text = stream.str();

            text_marker_array.markers.push_back(text_marker);
            prev_waypoint = waypoint;
        }
    }
    return text_marker_array;
}

//returns MarkerArray for deleting all markers
visualization_msgs::MarkerArray delete_all_markers()
{
    visualization_msgs::MarkerArray delete_markers_array;
    visualization_msgs::Marker delete_marker;
    delete_marker.id = 0;
    delete_marker.ns = "delete_marker";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_markers_array.markers.push_back(delete_marker);
    return delete_markers_array;
}

namespace ipp
{

    class PlannerVisualizer
    {
    protected:
        //ros things
        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        std::string local_frame = "local_enu";
        // publishers
        ros::Publisher exploration_final_path;
        // ros::Publisher final_path_views;
        ros::Publisher keep_out_zones;
        ros::Publisher planner_boundary;
        ros::Publisher waypoint_visualizer;
        ros::Publisher waypoint_num_visualizer;


    public:
        explicit PlannerVisualizer(ros::NodeHandle &nh, ros::NodeHandle pnh) : nh(nh), pnh(pnh)
        {
            visualization_scale = ros_utils::get_param<double>(pnh, "visualization_scale", 1.0);
            exploration_final_path = nh.advertise<visualization_msgs::Marker>("global_path/final_path", 10);
            // final_path_views = nh.advertise<visualization_msgs::Marker>("/global_path/final_path_frustums", 10);
            keep_out_zones = nh.advertise<visualization_msgs::Marker>("global_path/keep_out_zones", 10);
            planner_boundary = nh.advertise<visualization_msgs::Marker>("global_path/planner_boundary", 10);
            waypoint_visualizer = nh.advertise<visualization_msgs::Marker>("global_path/waypoints", 10);
            waypoint_num_visualizer = nh.advertise<visualization_msgs::MarkerArray>("global_path/waypoint_nums", 10);
        }

        /**
         * @brief Visualize the final path and frustums. Suggested to be overriden, make sure to call this base class method with PlannerVisualizer::visualize(...)
         *
         * @param ipp_planner
         */
        virtual void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map)
        {
            // exploration_final_path.publish(visualize_final_path(ipp_planner.get_best_path(), ipp_planner.XYZPsi_Space, local_frame, ipp_planner.desired_speed, ipp_planner.max_kappa));
            exploration_final_path.publish(visualize_dense_path(dense_path, local_frame));
            // final_path_views.publish(visualize_final_path_views(ipp_planner.get_best_path(), ipp_planner.sensor_params, local_frame));
            keep_out_zones.publish(visualize_keep_out_2d(local_frame, info_map.keep_out_zones));
            planner_boundary.publish(visualize_planner_boundary(info_map.bounds, local_frame));
            //visualize start pose of plan and subsequent setpoints
            waypoint_visualizer.publish(visualize_setpoints(local_frame, pose_to_plan_from, dense_path, waypoint_map));
            visualization_msgs::MarkerArray marker_num_msg = visualize_waypoint_nums(local_frame, dense_path, waypoint_map);
            //delete old markers
            waypoint_num_visualizer.publish(delete_all_markers());
            waypoint_num_visualizer.publish(marker_num_msg);
        }

        virtual void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from)
        {
            exploration_final_path.publish(visualize_final_path(ipp_planner.get_best_path(), ipp_planner.XYZPsi_Space, local_frame, ipp_planner.desired_speed, ipp_planner.max_kappa));
            // final_path_views.publish(visualize_final_path_views(ipp_planner.get_best_path(), ipp_planner.sensor_params, local_frame));
            planner_boundary.publish(visualize_planner_boundary(info_map.bounds, local_frame));
        }
    };

    class GreedyTrackVisualizer : public PlannerVisualizer
    {

    public:
        GreedyTrackVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh) : PlannerVisualizer(nh, pnh)
        {
        }

        void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map) override
        {
            PlannerVisualizer::vis_final(ipp_planner, info_map, pose_to_plan_from, dense_path, waypoint_map);

            const GreedyTrackPlanner &greedy_track = dynamic_cast<GreedyTrackPlanner &>(ipp_planner);
        }

        void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from) override
        {
            PlannerVisualizer::vis_while_planning(ipp_planner, info_map, pose_to_plan_from);

            const GreedyTrackPlanner &greedy_track = dynamic_cast<GreedyTrackPlanner &>(ipp_planner);
        }
    };

    class GreedySearchVisualizer : public PlannerVisualizer
    {
        ros::Publisher sampled_points;
        ros::Publisher view_points; 
        ros::Publisher copy_map_viz;

    public:
        GreedySearchVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh) : PlannerVisualizer(nh, pnh)
        {
            sampled_points = nh.advertise<visualization_msgs::MarkerArray>("global_path/random_samples", 10);
            view_points = nh.advertise<visualization_msgs::MarkerArray>("global_path/view_points", 10);
            copy_map_viz = nh.advertise<visualization_msgs::Marker>("global_path/copy_map_viz", 10);
        }

        void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map) override
        {
            PlannerVisualizer::vis_final(ipp_planner, info_map, pose_to_plan_from, dense_path, waypoint_map);

            const GreedySearchPlanner &greedy_search = dynamic_cast<GreedySearchPlanner &>(ipp_planner);

            std::vector<std::vector<double>> greedy_points = greedy_search.get_sampled_points();
            std::vector<std::vector<double>> greedy_view_points = greedy_search.get_view_points();
            visualization_msgs::Marker search_map_marker = visualize_map(greedy_search.local_map, true, "local_enu", 0.3);

            sampled_points.publish(visualize_points(greedy_points, local_frame, {1.0, 1.0, 0.0, 0.0}));
            view_points.publish(visualize_points(greedy_view_points, local_frame, {1.0, 0.0, 1.0, 0.0}));
            copy_map_viz.publish(search_map_marker);
            
        }

        void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from) override
        {
            PlannerVisualizer::vis_while_planning(ipp_planner, info_map, pose_to_plan_from);

            const GreedySearchPlanner &greedy_search = dynamic_cast<GreedySearchPlanner &>(ipp_planner);

            std::vector<std::vector<double>> greedy_points = greedy_search.get_sampled_points();
            std::vector<std::vector<double>> greedy_view_points = greedy_search.get_view_points();
            visualization_msgs::Marker search_map_marker = visualize_map(greedy_search.local_map, true, "local_enu", 0.3);

            sampled_points.publish(visualize_points(greedy_points, local_frame, {1.0, 1.0, 0.0, 0.0}));
            view_points.publish(visualize_points(greedy_view_points, local_frame, {1.0, 0.0, 1.0, 0.0}));
            copy_map_viz.publish(search_map_marker);
        }
    };

    class RandomVisualizer : public PlannerVisualizer
    {

    public:
        RandomVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh) : PlannerVisualizer(nh, pnh)
        {
        }

        void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map) override
        {
            PlannerVisualizer::vis_final(ipp_planner, info_map, pose_to_plan_from, dense_path, waypoint_map);

            const RandomTrackPlanner &random = dynamic_cast<RandomTrackPlanner &>(ipp_planner);
        }

        void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from) override
        {
            PlannerVisualizer::vis_while_planning(ipp_planner, info_map, pose_to_plan_from);

            const RandomTrackPlanner &random = dynamic_cast<RandomTrackPlanner &>(ipp_planner);
        }
    };

    class RandomSearchVisualizer : public PlannerVisualizer
    {
        ros::Publisher sampled_points;
        ros::Publisher view_points;

    public:
        RandomSearchVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh) : PlannerVisualizer(nh, pnh)
        {
            sampled_points = nh.advertise<visualization_msgs::MarkerArray>("global_path/random_samples", 10);
            view_points = nh.advertise<visualization_msgs::MarkerArray>("global_path/view_points", 10);
        }

        void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map) override
        {
            PlannerVisualizer::vis_final(ipp_planner, info_map, pose_to_plan_from, dense_path, waypoint_map);

            const RandomSearchPlanner &random_search = dynamic_cast<RandomSearchPlanner &>(ipp_planner);

            sampled_points.publish(visualize_points(random_search.get_sampled_points(), local_frame, {1.0, 1.0, 0.0, 0.0}));
            view_points.publish(visualize_points(random_search.get_view_points(), local_frame, {1.0, 0.0, 1.0, 0.0}));
        }

        void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from) override
        {
            PlannerVisualizer::vis_while_planning(ipp_planner, info_map, pose_to_plan_from);

            const RandomSearchPlanner &random = dynamic_cast<RandomSearchPlanner &>(ipp_planner);
        }
    };

    class TigrisVisualizer : public PlannerVisualizer
    {
        // ros::Publisher counterdetect_cyl;

        ros::Publisher final_path_observations;
        ros::Publisher exploration_arrows;
        ros::Publisher exploration_spheres;
        ros::Publisher exploration_tree;

        ros::Publisher sampled_points;
        ros::Publisher info_gain_text; // numeric text of the information gain

    public:
        TigrisVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh) : PlannerVisualizer(nh, pnh)
        {
            // counterdetect_cyl = nh.advertise<visualization_msgs::Marker>("/global_path/counterdetect_cyl", 100);
            final_path_observations = nh.advertise<visualization_msgs::Marker>("global_path/final_path_observations", 10);
            exploration_arrows = nh.advertise<visualization_msgs::MarkerArray>("global_path/arrows", 10);
            exploration_spheres = nh.advertise<visualization_msgs::Marker>("global_path/nodes", 10);
            exploration_tree = nh.advertise<visualization_msgs::Marker>("global_path/tree", 10);
            sampled_points = nh.advertise<visualization_msgs::Marker>("global_path/sampled_points", 10);
            info_gain_text = nh.advertise<visualization_msgs::MarkerArray>("global_path/info_gain_text", 10);
        }

        void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map) override
        {
            PlannerVisualizer::vis_final(ipp_planner, info_map, pose_to_plan_from, dense_path, waypoint_map);

            const Tigris &tigris = dynamic_cast<Tigris &>(ipp_planner);
            // for(size_t i=0; i<target_prior.size(); i++)
            // {
            //     counterdetect_cyl.publish(visualize_counter_detect_2d(i, local_frame, counter_detect_radius, target_prior));
            // }
            exploration_arrows.publish(visualize_nodes_arrow<TreeNode>(tigris.get_all_nodes(), local_frame));
            exploration_spheres.publish(visualize_nodes_sphere(tigris.get_all_nodes(), local_frame));
            exploration_tree.publish(visualize_tree_dubins<TreeNode>(tigris.get_all_nodes(), tigris.get_space_information_ptr(), local_frame));
            auto [sample_points, sample_points_text] = visualize_sampled_points(tigris.get_sampled_xyzi(), local_frame);
            sampled_points.publish(sample_points);
            info_gain_text.publish(delete_all_markers());
            info_gain_text.publish(sample_points_text);
        }

        void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from) override
        {
            PlannerVisualizer::vis_while_planning(ipp_planner, info_map, pose_to_plan_from);

            const Tigris &tigris = dynamic_cast<Tigris &>(ipp_planner);
            // for(size_t i=0; i<target_prior.size(); i++)
            // {
            //     counterdetect_cyl.publish(visualize_counter_detect_2d(i, local_frame, counter_detect_radius, target_prior));
            // }
            final_path_observations.publish(
                visualize_final_path_observations(
                    tigris.get_leaf_node_of_best_path(), ipp_planner.desired_speed, ipp_planner.sensor_params, info_map.observation_discretization_distance, local_frame));
            exploration_arrows.publish(visualize_nodes_arrow<TreeNode>(tigris.get_all_nodes(), local_frame));
            exploration_spheres.publish(visualize_nodes_sphere(tigris.get_all_nodes(), local_frame));
            exploration_tree.publish(visualize_tree_dubins<TreeNode>(tigris.get_all_nodes(), tigris.get_space_information_ptr(), local_frame));
            auto [sample_points, sample_points_text] = visualize_sampled_points(tigris.get_sampled_xyzi(), local_frame);
            sampled_points.publish(sample_points);
            info_gain_text.publish(sample_points_text);
        }
    };

    class PrimTreeVisualizer : public PlannerVisualizer
    {
        // ros::Publisher counterdetect_cyl;

        ros::Publisher final_path_observations;
        ros::Publisher exploration_arrows;
        ros::Publisher exploration_spheres;
        ros::Publisher exploration_tree;

        ros::Publisher info_gain_text; // numeric text of the information gain

    public:
        PrimTreeVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh) : PlannerVisualizer(nh, pnh)
        {
            // counterdetect_cyl = nh.advertise<visualization_msgs::Marker>("/global_path/counterdetect_cyl", 100);
            final_path_observations = nh.advertise<visualization_msgs::Marker>("global_path/final_path_observations", 10);
            exploration_arrows = nh.advertise<visualization_msgs::MarkerArray>("global_path/arrows", 10);
            exploration_spheres = nh.advertise<visualization_msgs::Marker>("global_path/nodes", 10);
            exploration_tree = nh.advertise<visualization_msgs::Marker>("global_path/tree", 10);
        }

        void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map) override
        {
            PlannerVisualizer::vis_final(ipp_planner, info_map, pose_to_plan_from, dense_path, waypoint_map);

            const PrimTree &primtree = dynamic_cast<PrimTree &>(ipp_planner);
            // for(size_t i=0; i<target_prior.size(); i++)
            // {
            //     counterdetect_cyl.publish(visualize_counter_detect_2d(i, local_frame, counter_detect_radius, target_prior));
            // }
            exploration_arrows.publish(visualize_nodes_arrow<TreeNode>(primtree.get_all_nodes(), local_frame));
            exploration_spheres.publish(visualize_nodes_sphere(primtree.get_all_nodes(), local_frame));
            exploration_tree.publish(visualize_tree_dubins<TreeNode>(primtree.get_all_nodes(), primtree.get_space_information_ptr(), local_frame));
        }

        void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from) override
        {
            PlannerVisualizer::vis_while_planning(ipp_planner, info_map, pose_to_plan_from);

            const PrimTree &primtree = dynamic_cast<PrimTree &>(ipp_planner);
            // for(size_t i=0; i<target_prior.size(); i++)
            // {
            //     counterdetect_cyl.publish(visualize_counter_detect_2d(i, local_frame, counter_detect_radius, target_prior));
            // }
            final_path_observations.publish(
                visualize_final_path_observations(
                    primtree.get_leaf_node_of_best_path(), ipp_planner.desired_speed, ipp_planner.sensor_params, info_map.observation_discretization_distance, local_frame));
            exploration_arrows.publish(visualize_nodes_arrow<TreeNode>(primtree.get_all_nodes(), local_frame));
            exploration_spheres.publish(visualize_nodes_sphere(primtree.get_all_nodes(), local_frame));
            exploration_tree.publish(visualize_tree_dubins<TreeNode>(primtree.get_all_nodes(), primtree.get_space_information_ptr(), local_frame));
        }
    };

    class PrimTreeBnBVisualizer : public PlannerVisualizer
    {
        // ros::Publisher counterdetect_cyl;

        ros::Publisher final_path_observations;
        ros::Publisher exploration_arrows;
        ros::Publisher exploration_spheres;
        ros::Publisher exploration_tree;

        ros::Publisher info_gain_text; // numeric text of the information gain

    public:
        PrimTreeBnBVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh) : PlannerVisualizer(nh, pnh)
        {
            // counterdetect_cyl = nh.advertise<visualization_msgs::Marker>("/global_path/counterdetect_cyl", 100);
            final_path_observations = nh.advertise<visualization_msgs::Marker>("global_path/final_path_observations", 10);
            exploration_arrows = nh.advertise<visualization_msgs::MarkerArray>("global_path/arrows", 10);
            exploration_spheres = nh.advertise<visualization_msgs::Marker>("global_path/nodes", 10);
            exploration_tree = nh.advertise<visualization_msgs::Marker>("global_path/tree", 10);
        }

        void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map) override
        {
            PlannerVisualizer::vis_final(ipp_planner, info_map, pose_to_plan_from, dense_path, waypoint_map);

            const PrimTreeBnB &primtreebnb = dynamic_cast<PrimTreeBnB &>(ipp_planner);
            // for(size_t i=0; i<target_prior.size(); i++)
            // {
            //     counterdetect_cyl.publish(visualize_counter_detect_2d(i, local_frame, counter_detect_radius, target_prior));
            // }
            exploration_arrows.publish(visualize_nodes_arrow<TreeNode>(primtreebnb.get_all_nodes(), local_frame));
            exploration_spheres.publish(visualize_nodes_sphere(primtreebnb.get_all_nodes(), local_frame));
            exploration_tree.publish(visualize_tree_dubins<TreeNode>(primtreebnb.get_all_nodes(), primtreebnb.get_space_information_ptr(), local_frame));
        }

        void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from) override
        {
            PlannerVisualizer::vis_while_planning(ipp_planner, info_map, pose_to_plan_from);

            const PrimTreeBnB &primtreebnb = dynamic_cast<PrimTreeBnB &>(ipp_planner);
            // for(size_t i=0; i<target_prior.size(); i++)
            // {
            //     counterdetect_cyl.publish(visualize_counter_detect_2d(i, local_frame, counter_detect_radius, target_prior));
            // }
            final_path_observations.publish(
                visualize_final_path_observations(
                    primtreebnb.get_leaf_node_of_best_path(), ipp_planner.desired_speed, ipp_planner.sensor_params, info_map.observation_discretization_distance, local_frame));
            exploration_arrows.publish(visualize_nodes_arrow<TreeNode>(primtreebnb.get_all_nodes(), local_frame));
            exploration_spheres.publish(visualize_nodes_sphere(primtreebnb.get_all_nodes(), local_frame));
            exploration_tree.publish(visualize_tree_dubins<TreeNode>(primtreebnb.get_all_nodes(), primtreebnb.get_space_information_ptr(), local_frame));
        }
    };

    class TrackingVisualizer : public PlannerVisualizer
    {
        ros::Publisher exploration_tree;
        ros::Publisher coverage_boundary_vis_pub;

    public:
        TrackingVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh) : PlannerVisualizer(nh, pnh)
        {
            exploration_tree = nh.advertise<visualization_msgs::Marker>("global_path/nodes", 10);
        }

        void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map) override
        {
            PlannerVisualizer::vis_final(ipp_planner, info_map, pose_to_plan_from, dense_path, waypoint_map);

            const TrackPlan &tracking = dynamic_cast<TrackPlan &>(ipp_planner);
            exploration_tree.publish(visualize_tree_line(tracking.get_all_nodes(), local_frame));
        }

        void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from) override
        {
            PlannerVisualizer::vis_while_planning(ipp_planner, info_map, pose_to_plan_from);

            const TrackPlan &tracking = dynamic_cast<TrackPlan &>(ipp_planner);
            exploration_tree.publish(visualize_tree_line(tracking.get_all_nodes(), local_frame));
        }
    };

    class CoverageVisualizer : public PlannerVisualizer
    {
    public:
        CoverageVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh) : PlannerVisualizer(nh, pnh)
        {
        }

        void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map) override
        {
            PlannerVisualizer::vis_final(ipp_planner, info_map, pose_to_plan_from, dense_path, waypoint_map);

            const CoveragePlanner &coverage = dynamic_cast<CoveragePlanner &>(ipp_planner);
        }

        void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from) override
        {
            PlannerVisualizer::vis_while_planning(ipp_planner, info_map, pose_to_plan_from);

            const CoveragePlanner &coverage = dynamic_cast<CoveragePlanner &>(ipp_planner);
        }
    };

    class MCTSSearchVisualizer : public PlannerVisualizer
    {
        ros::Publisher final_path_observations;
        ros::Publisher exploration_arrows;
        ros::Publisher exploration_tree;
        ros::Publisher info_gain_text; // numeric text of the information gain
        ros::Publisher avg_value_text; // numeric text of the average value

        ros::Publisher rollout_frustums;

    public:
        MCTSSearchVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh) : PlannerVisualizer(nh, pnh)
        {
            exploration_arrows = nh.advertise<visualization_msgs::MarkerArray>("global_path/arrows", 10);
            exploration_tree = nh.advertise<visualization_msgs::Marker>("global_path/tree", 10);
            final_path_observations = nh.advertise<visualization_msgs::Marker>("global_path/final_path_observations", 10);
            info_gain_text = nh.advertise<visualization_msgs::MarkerArray>("global_path/info_gain_text", 10);
            avg_value_text = nh.advertise<visualization_msgs::MarkerArray>("global_path/mcts_search_avg_value_text", 10);
        }
        void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map) override
        {
            PlannerVisualizer::vis_final(ipp_planner, info_map, pose_to_plan_from, dense_path, waypoint_map);
            MCTSSearch &mcts = dynamic_cast<MCTSSearch &>(ipp_planner);

            auto the_nodes = mcts.get_all_nodes(1);

            exploration_tree.publish(visualize_tree_dubins<MCTSNode>(the_nodes, mcts.get_space_information_ptr(), local_frame));
        }

        void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from) override
        {
            PlannerVisualizer::vis_while_planning(ipp_planner, info_map, pose_to_plan_from);
            MCTSSearch &mcts = dynamic_cast<MCTSSearch &>(ipp_planner);

            auto the_nodes = mcts.get_all_nodes(1);

            exploration_tree.publish(visualize_tree_dubins<MCTSNode>(the_nodes, mcts.get_space_information_ptr(), local_frame));
            info_gain_text.publish(visualize_node_info_gain_text<MCTSNode>(the_nodes, local_frame));
        }
    };

#ifdef USE_MCTS
    class MCTSVisualizer : public PlannerVisualizer
    {
        ros::Publisher final_path_observations;
        ros::Publisher exploration_arrows;
        ros::Publisher exploration_tree;
        ros::Publisher info_gain_text; // numeric text of the information gain
        ros::Publisher avg_value_text; // numeric text of the average value

        ros::Publisher rollout_frustums;

    public:
        MCTSVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh) : PlannerVisualizer(nh)
        {
            exploration_arrows = nh.advertise<visualization_msgs::MarkerArray>("global_path/arrows", 10);
            exploration_tree = nh.advertise<visualization_msgs::Marker>("global_path/tree", 10);
            final_path_observations = nh.advertise<visualization_msgs::Marker>("global_path/final_path_observations", 10);
            info_gain_text = nh.advertise<visualization_msgs::MarkerArray>("global_path/info_gain_text", 10);
            avg_value_text = nh.advertise<visualization_msgs::MarkerArray>("global_path/mcts_avg_value_text", 10);
        }
        void vis_final(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from, planner_map_interfaces::Plan dense_path, std::vector<int> waypoint_map) override
        {
            PlannerVisualizer::vis_final(ipp_planner, info_map, pose_to_plan_from, dense_path);
            MCTS &mcts = dynamic_cast<MCTS &>(ipp_planner);

            // auto the_nodes = ros_utils::get_param<bool>(this->nh, "vis_while_planning") ?  mcts.get_expanded_nodes() : mcts.get_all_nodes();
            // auto the_nodes = mcts.get_expanded_nodes();
            auto the_nodes = mcts.get_all_nodes(1);

            // below is probably super expensive, only uncomment for debugging
            // final_path_observations.publish(visualize_final_path_observations(mcts.get_leaf_node_of_best_path(), ipp_planner.sensor_params, local_frame));
            // exploration_arrows.publish(visualize_nodes_arrow<MCTSNode>(the_nodes, local_frame));
            exploration_tree.publish(visualize_tree_dubins<MCTSNode>(the_nodes, mcts.get_space_information_ptr(), local_frame));
            // info_gain_text.publish(visualize_node_info_gain_text<MCTSNode>(the_nodes, local_frame));
            // avg_value_text.publish(visualize_node_avg_value_text(the_nodes, local_frame));
        }

        void vis_while_planning(Planner &ipp_planner, InfoMap &info_map, std::vector<double> pose_to_plan_from) override
        {
            PlannerVisualizer::vis_while_planning(ipp_planner, info_map, pose_to_plan_from);
            MCTS &mcts = dynamic_cast<MCTS &>(ipp_planner);

            // auto the_nodes = ros_utils::get_param<bool>(this->nh, "vis_while_planning") ?  mcts.get_expanded_nodes() : mcts.get_all_nodes();
            // auto the_nodes = mcts.get_expanded_nodes();
            auto the_nodes = mcts.get_all_nodes(1);

            // below is probably super expensive, only uncomment for debugging
            final_path_observations.publish(visualize_final_path_observations(mcts.get_leaf_node_of_best_path(), ipp_planner.desired_speed, ipp_planner.sensor_params, info_map.observation_discretization_distance, local_frame));
            // exploration_arrows.publish(visualize_nodes_arrow<MCTSNode>(the_nodes, local_frame));
            exploration_tree.publish(visualize_tree_dubins<MCTSNode>(the_nodes, mcts.get_space_information_ptr(), local_frame));
            info_gain_text.publish(visualize_node_info_gain_text<MCTSNode>(the_nodes, local_frame));
            avg_value_text.publish(visualize_node_avg_value_text(the_nodes, local_frame));
        }
    };
#endif

}
