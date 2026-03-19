#!/usr/bin/env python2

import numpy as np
from std_msgs.msg import UInt32
from planner_map_interfaces.msg import Plan
from nav_msgs.msg import Odometry
import rospy


plan_msg = None
current_target_waypoint = 0
last_distance = None


def update_current_plan_msg(plan_msg_):
    rospy.loginfo("Waypoint manager received Plan Msg")
    global plan_msg, current_target_waypoint
    plan_msg = plan_msg_
    current_target_waypoint = 0


def update_target_waypoint(odom_msg):
    # rospy.loginfo("Waypoint manager hi")
    if plan_msg is not None:
        global current_target_waypoint, last_distance
        p = odom_msg.pose.pose.position

        target_p = plan_msg.plan[current_target_waypoint].position.position            
        dist = np.sqrt((p.x - target_p.x)**2 + (p.y - target_p.y) ** 2 + (p.z - target_p.z)**2)

        # print("odom:", p)
        # print("target:", target_p)
        # print("current idx", current_target_waypoint)

        if last_distance is None or abs(dist - last_distance) > 2:
            rospy.logdebug("Current agent position: {}".format(p))
            rospy.logdebug("Current target waypoint: {}".format(target_p))
            rospy.logdebug("Distance to target waypoint {}: {}".format(current_target_waypoint, dist))
            last_distance = dist
        if dist <= rospy.get_param("ipp_planners_node/waypoint_threshold"):
            rospy.loginfo("WAYPOINT %d REACHED, now targeting waypoint %d" % (current_target_waypoint, current_target_waypoint+1))
            current_target_waypoint += 1


if __name__ == "__main__":
    rospy.init_node("planner_simple_waypoint_manager")
    rospy.sleep(1)
    if rospy.get_param("ipp_planners_node/use_own_waypoint_manager"):
        rospy.loginfo("Using own waypoint manager")
        waypoint_num_pub = rospy.Publisher('waypoint_num', UInt32, queue_size=10)
        plan_sub = rospy.Subscriber("global_path", Plan, update_current_plan_msg, queue_size=1)
        agent_odom_sub = rospy.Subscriber(rospy.get_param("ipp_planners_node/agent_odom_topic"), Odometry, update_target_waypoint, queue_size=1)

        rate = rospy.Rate(100) 

        while not rospy.is_shutdown():
            waypoint_num_pub.publish(current_target_waypoint)
            rate.sleep()
    else:
        # rospy.loginfo("Simple waypoint manager exiting because ipp_planners_node/use_own_waypoint_manager is false. This is fine as long as a different waypoint manager is responsible.")
        pass
