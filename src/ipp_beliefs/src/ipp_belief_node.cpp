/**
 * @file ipp_belief_node.cpp
 * @author your name (you@domain.com)
 * @brief Subscribes to the simple sim and updates the propagated targets
 * @version 0.1
 * @date 2022-03-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <ros/ros.h>
#include <memory>
#include <vector>
#include <iostream>

#include "ipp_belief/ipp_belief_node.h"

int main(int argc, char **argv)
{
    srand(2204);
    ros::init(argc, argv, "ipp_belief_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    IppBeliefNode ipp_belief_node(nh,pnh);
    // ipp_belief_node.service_add_priors = nh.advertiseService(
    //     "/ipp_belief/add_priors_srv", &IppBeliefNode::add_priors_callback, &ipp_belief_node);
    std::cout << "IppBeliefNode created" << std::endl;

    ipp_belief_node.run();
}