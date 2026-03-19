#include "ros/ros.h"
#include "std_msgs/String.h"
#include "planner_map_interfaces/Plan.h"
#include "planner_map_interfaces/PlanRequest.h"
#include "planner_map_interfaces/SensorConstraint.h"
#include "planner_map_interfaces/TargetPrior.h"
#include "planner_map_interfaces/Waypoint.h"
#include "geographic_msgs/GeoPose.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include <sstream>

ros::Publisher plan_pub;

void labelsTensorCallback()
{
    ROS_INFO("I received a LabelsTensor msg");
}

void sensorConstraintCallback(const planner_map_interfaces::SensorConstraint::ConstPtr& msg)
{
    ROS_INFO("I received a Sensor Constraint msg");
}

void waypointCallback()
{
    ROS_INFO("I received a Waypoint msg");
}

planner_map_interfaces::Waypoint setSampleWaypoint(const planner_map_interfaces::PlanRequest::ConstPtr& msg)
{
    planner_map_interfaces::Waypoint waypoint;
    
    waypoint.stamp.frame_id = msg->header.frame_id;
    waypoint.stamp.stamp = ros::Time::now();
    
    waypoint.waypoint_type = 1;

    int bound_count = msg->search_bounds.points.size();
    float lat_sum = 0;
    float lon_sum = 0;

    for (int i = 0; i < bound_count; i++){
      lat_sum += msg->search_bounds.points[i].y;
      lon_sum += msg->search_bounds.points[i].x;
    }
    
    waypoint.position.position.y = lat_sum/bound_count;
    waypoint.position.position.x = lon_sum/bound_count;
    waypoint.position.position.z = msg->start_pose.position.z+20;
    
    waypoint.position.orientation.x = 0.0;
    waypoint.position.orientation.y = 0.0;
    waypoint.position.orientation.z = 0.0;
    waypoint.position.orientation.w = 1.0;
    
    waypoint.command_speed = msg->desired_speed;


    // Set gimbal 
    waypoint.direction.x = 0.0;
    waypoint.direction.y = 0.0;
    waypoint.direction.z = 0.0;

    waypoint.field_of_view_horizontal = 1.0;
    waypoint.field_of_view_vertical = 1.0;

    return waypoint;
}

void PlanRequestCallback(const planner_map_interfaces::PlanRequest::ConstPtr& msg)
{
    ROS_INFO("I received a PlanRequest msg");
    ROS_INFO("Planning for %.2f seconds", msg->max_planning_time);

    ros::Duration(msg->max_planning_time).sleep();

    planner_map_interfaces::Plan plan_msg;

    plan_msg.header.seq = msg->header.seq;
    plan_msg.header.stamp = ros::Time::now();
    
    plan_msg.plan.push_back(setSampleWaypoint(msg));

    plan_pub.publish(plan_msg);
    ROS_INFO("Published Plan message");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_map_interfaces_node");
    ros::NodeHandle n;
    plan_pub = n.advertise<planner_map_interfaces::Plan>("plan_response", 1000);
    ros::Subscriber plan_sub = n.subscribe("plan_request", 1000, PlanRequestCallback);

    ros::Subscriber sensor_constraint_sub = n.subscribe("sensor_constraint", 1, sensorConstraintCallback);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
