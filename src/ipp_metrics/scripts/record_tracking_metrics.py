#!/usr/bin/env python2
"""
Path overlap with octomap
# needs to be C++ API then oh.
"""

import rospy

from std_msgs.msg import Float32, Float64MultiArray, String
from planner_map_interfaces.msg import PlanRequest, Plan
import os
import time
# import plot_metrics

global count
global count_ind

count = 0
count_ind = 0
timestr = ""
seq = 0
received_seq = False
totalVarFileName = ""
indVarFileName = ""

def receivedPlan(msg):
    # print('!!!!receivedPlan callback reached!!!!')
    #Set to true to start recording
    global received_seq
    received_seq = True

def getSeqId(msg):
    # print('!!!!getSeqId callback reached!!!!')
    #Reset everything and create new csv filename
    global count
    count = 0

    global count_ind
    count_ind = 0
    
    #Get seqID for exp name
    ## print(msg)
    # print(msg.scenario)
    # print(type(msg.scenario))
    global seq
    seq = msg.scenario

    #Plot and save fig for this run (unless it is first time -> received_seq = False)
    global received_seq
    #global totalVarFileName
    #global indVarFileName
    #We have planned at least once so files exist
    #NO MORE PLOTTING SINCE WE DO NOT USE AND MAY CONFLICT TIME-WISE
    # if received_seq:
    #     plot_metrics.plot_total_vars(totalVarFileName)
    #     plot_metrics.plot_individual_vars(indVarFileName)

    #Reset everything and create new csv filename
    # global count
    # count = 0

    # global count_ind
    # count_ind = 0

    planner_name = rospy.get_param("/ipp_planners_node/planner")

    global timestr
    timestr = time.strftime("%Y%m%d-%H%M%S") + "_" + planner_name + "_"

    received_seq = False

def record_variance(variance_msg):
    # print('!!!recordVariance reached!!!')
    global count
    global received_seq
    global seq
    global totalVarFileName

    stamp = rospy.Time.now()

    if count < 700 and variance_msg.data > 0 and received_seq:  # only record this many messages (i.e. seconds)
        totalVarFileName = timestr + "seq_" + str(seq) + "_variance.csv"
        if count == 0:
            with open(totalVarFileName, "a") as f:
                f.write("time,variance\n")
        rospy.loginfo("Recording variance " + str(variance_msg.data))
        with open(totalVarFileName, "a") as f:
            f.write(str(stamp.secs) + "," + str(variance_msg.data) + "\n")
        count+=1
    else:
        rospy.loginfo("No recording of total variance this iteration!")

def record_individual_variance(individual_variance_msg):
    # print('!!!reached recordIndividualVariance!!!')
    global count_ind
    global received_seq
    global seq
    global indVarFileName

    stamp = rospy.Time.now()

    if count_ind < 700 and len(individual_variance_msg.data) > 0 and received_seq:
        indVarFileName = timestr + "seq_" + str(seq) + "_individual_variance.csv"
        if count_ind == 0:
            with open(indVarFileName, "a") as f:
                f.write("time")
                # print(len(individual_variance_msg.data))
                for i in range(len(individual_variance_msg.data)):
                    f.write(",variance" + str(i))
                f.write("\n")
        rospy.loginfo("Recording variance " + str(individual_variance_msg.data))
        ## print(type(individual_variance_msg.data))
        with open(indVarFileName, "a") as f:
            f.write(str(stamp.secs))
            for i in range(len(individual_variance_msg.data)):
                f.write("," + str(individual_variance_msg.data[i]))
            f.write("\n")
        count_ind+=1
    else:
        rospy.loginfo("No recording of individual variance this iteration!")


if __name__ == "__main__":
    cwd = os.getcwd()
    # print(cwd)
    # We execute the commented code only when we receive a plan request
    # planner_name = rospy.get_param("/ipp_planners_node/planner")
    # global timestr
    # timestr = time.strftime("%Y%m%d-%H%M%S") + "_" + planner_name + "_"


    rospy.init_node("metrics_node", anonymous=True)
    rospy.Subscriber("/ipp_belief/total_variance", Float32, record_variance)
    rospy.Subscriber("/ipp_belief/variance_array", Float64MultiArray, record_individual_variance)
    rospy.Subscriber("/planner/plan_request", PlanRequest, getSeqId)
    rospy.Subscriber("/global_path", Plan, receivedPlan)
        
    rate = rospy.Rate(2)  
    while not rospy.is_shutdown():
        rate.sleep()
    
    # plot_metrics.plot_total_vars(totalVarFileName)
    # plot_metrics.plot_individual_vars(indVarFileName)
    # print('done')
