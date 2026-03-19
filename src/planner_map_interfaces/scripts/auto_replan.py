#!/usr/bin/python
"""
Event based auto replans.
"""
import rospy

from planner_map_interfaces.msg import Plan
import std_msgs.msg

variance = None
prev_variance = None
variance_of_last_replan = None

current_waypoint = None
total_waypoints = None

received_back_plan = None

def variance_callback(msg):
    """
    Save the current variance
    """
    global variance
    global prev_variance
    global variance_of_last_replan

    if variance is not None and abs(msg.data - variance) > 10:
        rospy.loginfo("Prev Variance: {}".format(prev_variance))
        rospy.loginfo("Variance: {}".format(variance))


    variance = msg.data
    # initialization only
    if prev_variance is None:
        prev_variance = variance
    if variance_of_last_replan is None:
        variance_of_last_replan = variance

def current_waypoint_callback(msg):
    """
    Save the current waypoint
    """
    global current_waypoint
    if msg.data != current_waypoint:
        rospy.loginfo("Current waypoint: {}".format(current_waypoint))
    current_waypoint = msg.data

def plan_callback(msg):
    """
    Save the total waypoints
    """
    global total_waypoints
    global received_back_plan
    if len(msg.plan) != total_waypoints:
        rospy.loginfo("Total waypoints: {}".format(total_waypoints))
    total_waypoints = len(msg.plan)
    received_back_plan = True



def main():
    rospy.init_node('auto_replan', anonymous=True)
    rate = rospy.Rate(1)
    rospy.loginfo("Started replan node. Will trigger replans for variance changes and end of plans")
    publisher = rospy.Publisher("/planner/replan", std_msgs.msg.String, queue_size=10, latch=True)
    #  rostopic pub -1 /planner/replan std_msgs/String "hi
    var_subscriber = rospy.Subscriber("/ipp_belief/total_variance", std_msgs.msg.Float32, callback=variance_callback)
    current_waypoint_sub = rospy.Subscriber("/simulator/waypoint_num", std_msgs.msg.UInt32, callback=current_waypoint_callback)
    plan_request_sub = rospy.Subscriber("/global_path", Plan, callback=plan_callback)
    global prev_variance
    global variance_of_last_replan
    global received_back_plan

    rospy.loginfo("Variance of last replan: {}".format(variance_of_last_replan))

    replan = False
    while True:

        if received_back_plan:
            # if the variance decreased a lot, then replan
            msg = ""
            if variance is not None and variance <= 0.8 * prev_variance:
                msg = "replan due to variance decrease of {}".format(variance - prev_variance)
                replan = True
            
            # if the variance increased a lot, then replan
            if variance is not None and variance_of_last_replan > 0 and variance - variance_of_last_replan > 1.5 * variance_of_last_replan:
                msg = "replan due to variance increase of {}".format(variance - variance_of_last_replan)
                replan = True
            
            # if the current waypoint is reaching the last waypoint, then replan
            if current_waypoint is not None and total_waypoints is not None and current_waypoint >= total_waypoints - 2:
                msg = "replan due to reaching the last waypoint"
                replan = True

            if replan:
                variance_of_last_replan = variance
                publisher.publish(msg)
                rospy.loginfo(msg)
                rospy.loginfo("Variance of last replan: {}".format(variance_of_last_replan))
                replan = False
                received_back_plan = False

        prev_variance = variance

        if rospy.is_shutdown():
            rospy.loginfo('shutdown')
            break
        rate.sleep()

if __name__ == '__main__':
    main()
    
