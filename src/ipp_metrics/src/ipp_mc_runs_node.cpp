
#include <experimental/filesystem>
#include <ros/package.h>
#include "ipp_metrics/ipp_mc_runs_node.h"
// #include "ipp_planners/ipp_planners_node.h"

using namespace ipp;
namespace fs = std::experimental::filesystem;

int main(int argc, char** argv){
    ros::init(argc, argv, "ipp_mc_runs_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    PlannerNodeMC node(nh, pnh);
    node.run_mc();
}

void PlannerNodeMC::run_mc()
{
    ROS_INFO_STREAM("Waiting in case of parameter updates");
    ros::Duration(5.0).sleep();
    ROS_WARN_STREAM("Done waiting");
    while (ros::ok())
    {
        // ROS_INFO_STREAM("startup " << this->received_sim_startup_msg);
        // ROS_INFO_STREAM("init_new " << this->init_new_planner_and_plan);
        if (init_new_planner_and_plan && this->received_sim_startup_msg) // delay for sim
        {
            if (planner_type_index == planner_types.size()-1)
            {
                planner_type_index = 0;
                loop_count++;

                //check to see if we finished all tests
                if (loop_count >= number_of_trials)
                {
                    //close log file for number of ground truth targets
                    if(this->gt_target_log_file.is_open())
                    {   
                        this->gt_target_log_file.close();
                    }
                    ROS_INFO("\n\nFinished all trials\n\n");
                    // kill_all_nodes_pub.publish(std_msgs::String());
                    ros::shutdown(); 
                    break;
                }

            }
            else
            {
                planner_type_index++;
            }


            ROS_WARN_STREAM("\n###############################\nPlanner: " 
                            << planner_types[planner_type_index] 
                            << " Trial: " 
                            << loop_count 
                            << "\n###############################");
            init_attributes();
            
            if (sample_plan_request_randomly)
            {
                // sample new plan (if on first planner) and publish it
                if (planner_type_index == 0)
                {
                    plan_request_msg = samplePlanRequest(generateSearchPriors());
                }
                plan_request_pub.publish(plan_request_msg);
                ROS_INFO("[PlannerNodeMC] Published plan request. Waiting for 4 seconds for simple sim to receive");
                ros::Duration(4.0).sleep();
                plan_request_callback(plan_request_msg); // Need to turn off if executing but not replanning. 
            }
            else if(first_plan_request_received)
            {
                ROS_INFO("[PlannerNodeMC] Publishing previous plan request again");
                // need to publish plan so that simple sim publishes teleports agent to original start
                plan_request_pub.publish(latest_plan_req_msg);
            }
            waypoint_num = -2; // reset waypoint num
            // increment the planner type index and loop counter
        }
        if (this->has_new_plan_request || this->is_time_to_replan())
        {
            ROS_WARN("Planning");
            // Update parameters from ros param server
            if (planner_types[planner_type_index] == "tigris")
            {
                Tigris &tigris = dynamic_cast<Tigris &>(*this->ipp_planner);
                tigris.extend_dist = get_param<double>(pnh, "extend_dist");
                tigris.extend_radius = get_param<double>(pnh, "extend_radius");
                tigris.prune_radius = get_param<double>(pnh, "prune_radius");

                if(this->horizon_length != 0.0)
                {
                    ROS_WARN_STREAM("Tigris extend_dist " << tigris.extend_dist << 
                                ", extend_radius " << tigris.extend_radius << 
                                ", prune_radius: " << tigris.prune_radius <<
                                ", horizon_length: " << this->horizon_length);
                }
                else
                {
                    ROS_WARN_STREAM("Tigris extend_dist " << tigris.extend_dist << 
                                ", extend_radius " << tigris.extend_radius << 
                                ", prune_radius: " << tigris.prune_radius);
                }
            }
            // Update parameters from ros param server
            if (planner_types[planner_type_index] == "tigris")
            {
                Tigris &tigris = dynamic_cast<Tigris &>(*this->ipp_planner);
                tigris.extend_dist = get_param<double>(pnh, "extend_dist");
                tigris.extend_radius = get_param<double>(pnh, "extend_radius");
                tigris.prune_radius = get_param<double>(pnh, "prune_radius");
            }
            else if (planner_types[planner_type_index] == "rect_coverage")
            {
                // set the sensor width different
                this->ipp_planner->viewpoint_goal = -0.3;
                ROS_ERROR_STREAM("Viewpoint goal hardcoded to " << this->ipp_planner->viewpoint_goal << " for metrics testing of coverage planner");
            }
            this->make_and_publish_plan();
            // check if empty plan
            if (this->current_plan_msg.plan.size() == 0)
            {
                ROS_WARN("[PlannerNodeMC] Planner returned empty plan");
                // init_new_planner_and_plan = true;
                // this->current_plan_msg.plan.clear();
                // this->current_plan_waypoint_map.clear();
            }
            this->time_of_last_replan = ros::Time::now();
            // if (this->should_visualize && this->current_plan_msg.plan.size() > 0)
            // {
            //     this->visualize();
            // } //moved to make_and_publish_plan()
            if (!execute_plan && this->received_sim_startup_msg)
            {
                ROS_WARN_STREAM("[PlannerNodeMC] Execute plan is off; starting next sim");
                init_new_planner_and_plan = true;
                this->current_plan_msg.plan.clear();
                this->current_plan_waypoint_map.clear();
            }
        }
        // if (this->should_visualize && this->current_plan_msg.plan.size() > 0)
        // {
        //     this->visualize();
        // } //moved to make_and_publish_plan()
        // Check for ending condition of if drone has reached the end of path
        // TODO also check condition if budget used up (greedy planner? or add to greedy)
        if (this->waypoint_num == this->current_plan_msg.plan.size() && 
                this->current_plan_msg.plan.size() > 0)
        {
            ROS_INFO("[PlannerNodeMC] Reached end of plan. Starting next sim");
            //resetting so we can start next sim
            init_new_planner_and_plan = true;
            this->current_plan_msg.plan.clear();
            this->current_plan_waypoint_map.clear();
            // close log file
            if (realtime_search_map_log_file.is_open())
            {
                this->realtime_search_map_log_file.close();
            }
            if(this->planner_log_file.is_open())
            {
                this->planner_log_file.close();
            }
            if(this->plan_log_file.is_open())
            {
                this->plan_log_file.close();
            }
            if(this->target_log_file.is_open())
            {
                this->target_log_file.close();
                this->detected_target_ids.clear();
            }
        }

        // Check if all of the targets are outside of the bounds
        // if (all_targets_outside_of_bounds())
        // {
        //     ROS_INFO("[PlannerNodeMC] All targets outside of bounds starting next sim");
        //     init_new_planner_and_plan = true;
        //     this->current_plan_msg.plan.clear();
        // }
        // Check if run sim is off
        double last_launch_time;
        nh.getParam("/ipp_planners_node/last_launch_time", last_launch_time);
        // ROS_WARN_STREAM("Last lauch time: " << last_launch_time);
        if (execute_plan && last_launch_time)
        {
            // TODO: log realtime search map metrics to file
            ros::Time stamp = ros::Time::now();
            std::stringstream ss;

            //write search map metrics to a file
            if(this->log_search_map_metrics)
            {
                this->realtime_search_map_log_file << std::fixed << ros::Time::now().toSec() << ","
                                               << average_entropy << ","
                                               << average_prob << std::endl;
            }

            //record number of targets detected to file
            if(this->sample_targets)
            {
                this->target_log_file << std::fixed << ros::Time::now().toSec() << ","
                                    << detected_target_ids.size() << std::endl;
            }
            
            // double max_budget = ros_utils::get_param<double>(nh, "max_budget");
            double max_budget = this->latest_plan_req_msg.maximum_range;
            double planned_run_time = max_budget / this->latest_plan_req_msg.desired_speed;
            double curr_time = ros::Time::now().toSec();
            // double last_launch_time = ros_utils::get_param<double>(nh, "/ipp_planners_node/last_launch_time");
            // ROS_WARN_STREAM("last launch time is " << last_launch_time);
            // ROS_WARN_STREAM("the drone has been flying for " << curr_time - last_launch_time << " seconds");
            if (curr_time - last_launch_time > planned_run_time)
            {
                ROS_WARN_STREAM("Times up, starting next sim");
                init_new_planner_and_plan = true;
                this->nh.setParam("/ipp_planners_node/last_launch_time", ros::Time::now().toSec());
                this->current_plan_msg.plan.clear();
                this->current_plan_waypoint_map.clear();
                // close log file
                if (realtime_search_map_log_file.is_open())
                {
                    this->realtime_search_map_log_file.close();
                }
                if(this->planner_log_file.is_open())
                {
                    this->planner_log_file.close();
                }
                if(this->plan_log_file.is_open())
                {
                    this->plan_log_file.close();
                }
                if(this->target_log_file.is_open())
                {
                    this->target_log_file.close();
                    this->detected_target_ids.clear();
                }
            }
        }
        // if (!execute_plan && this->received_sim_startup_msg)
        // {
        //     ROS_WARN_STREAM("[PlannerNodeMC] Execute plan is off; starting next sim");
        //     init_new_planner_and_plan = true;
        //     this->current_plan_msg.plan.clear();
        //     this->current_plan_waypoint_map.clear();
        // }
        ros::spinOnce();
        this->loop_rate.sleep();
    }
}

void PlannerNodeMC::init_attributes()
{
    pnh.setParam("planner", planner_types[planner_type_index]); // set the planner type
    pnh.setParam("scene_init", true);
    init_new_planner_and_plan = false;

    // Misc attributes
    if (planner_types[planner_type_index] == "greedy" || planner_types[planner_type_index] == "random")
    {
        double min_val = 5.0;
        double seconds_until_auto_replan = get_param<double>(pnh, "seconds_until_auto_replan");
        if (seconds_until_auto_replan < min_val)
        {
            seconds_until_auto_replan = min_val;
            ROS_WARN("Duration between replans set to %f because min value", seconds_until_auto_replan);
        }
        else
        {
            ROS_WARN("Duration between replans set to %f", seconds_until_auto_replan);
        }
        this->duration_between_replan = ros::Duration(seconds_until_auto_replan);
    }
    else if (planner_types[planner_type_index] == "rect_coverage")
    {
        this->duration_between_replan = ros::Duration(0.0);
        ROS_INFO_STREAM("Duration between replans set to " << this->duration_between_replan << " because coverage planner");
    }
    else
    {
        double seconds_until_auto_replan = get_param<double>(pnh, "seconds_until_auto_replan");
        ROS_INFO("Duration between replans set to %f", seconds_until_auto_replan);
        this->duration_between_replan = ros::Duration(seconds_until_auto_replan);
    }

    //if execute is true then we don't want to record metrics while planning. So, we set log_plan_metrics to false
    if(this->execute_plan && this->duration_between_replan > ros::Duration(0.0))
    {
        nh.setParam("ipp_planners_node/log_plan_metrics", false);
        bool temp_param;
        nh.getParam("ipp_planners_node/log_plan_metrics", temp_param);
        ROS_WARN_STREAM("ipp_planners_node/log_plan_metrics set to " << temp_param << " since execute_plan set to true");
    }

    //set up logging before we initialize everything so params are read in correctly
    std::string log_directory;
    nh.getParam("ipp_planners_node/plan_metrics_csv_directory", log_directory);
    this->log_plan_metrics = ros_utils::get_param<bool>(nh, "ipp_planners_node/log_plan_metrics", false);

    if(this->log_plan_metrics || this->log_search_map_metrics)
    {
        //set up file that we want to write planner metrics to. Need to set up here so it's read in when initializing planner
        std::string planner_log_filename = log_directory + planner_types[planner_type_index] + "_planner_metrics_" + std::to_string(loop_count+1) + ".csv";
        nh.setParam("ipp_planners_node/planner_metrics_csv_file", planner_log_filename);
        //set up file that we want to write final path to. Need to set up here so it's read in when initializing planner
        std::string final_plan_log_filename = log_directory + planner_types[planner_type_index] + "_final_path_" + std::to_string(loop_count+1) + ".csv";
        nh.setParam("ipp_planners_node/final_path_csv_file", final_plan_log_filename);
        ROS_INFO_STREAM("Writing final plan to " << final_plan_log_filename);
        if(execute_plan)
        {
            //record search metrics if search metrics flag is set
            if(this->log_search_map_metrics)
            {
                std::string search_map_log_filename = log_directory + planner_types[planner_type_index] + "_search_map_metrics_" + std::to_string(loop_count+1) + ".csv";
                ROS_INFO_STREAM("[PlannerNodeMC] logging realtime search map metrics to " << search_map_log_filename);
                this->realtime_search_map_log_file.open(search_map_log_filename, std::ios::out | std::ios::trunc);
                this->realtime_search_map_log_file << "time, average_entropy, average_prob" << std::endl;
            }
            //record custom search metrics if we're replanning
            if(this->duration_between_replan > ros::Duration(0.0))
            {
                //open file used to record planner metrics
                ROS_INFO_STREAM("[PlannerNodeMC] logging planner metrics to " << planner_log_filename);
                this->planner_log_file.open(planner_log_filename, std::ios::out | std::ios::trunc);
                this->planner_log_file << "time,path_info_gain,path_cost,num_waypoints,num_nodes_in_path,num_samples,tree_size" << std::endl;
            }
            //open file used to record path metrics
            ROS_INFO_STREAM("[PlannerNodeMC] Writing final path to " << final_plan_log_filename);
            this->plan_log_file.open(final_plan_log_filename, std::ios::out | std::ios::trunc);
            this->plan_log_file << "x,y,z,psi" << std::endl;
            //check to see if we want to randomly sample targets
            sample_targets = ros_utils::get_param<bool>(pnh, "sample_targets_from_prior", false);
            if(sample_targets)
            {
                std::string target_log_filename = log_directory + planner_types[planner_type_index] 
                    + "_detected_targets_" + std::to_string(loop_count+1) + ".csv";
                ROS_INFO_STREAM("[PlannerNodeMC] logging detected targets to " << target_log_filename);
                this->target_log_file.open(target_log_filename, std::ios::out | std::ios::trunc);
                this->target_log_file << "time, num_targets" << std::endl;
                //open file for ground truth targets if it isn't already open
                //put it here because we need this to happen BEFORE we make a plan request.
                if(!this->gt_target_log_file.is_open())
                {
                    std::string gt_target_log_filename = log_directory + "/ground_truth_targets.csv";
                    this->gt_target_log_file.open(gt_target_log_filename, std::ios::out | std::ios::trunc);
                    this->gt_target_log_file << "planner,loop_count,num_targets" << std::endl;
                }
            }
        }
    }
    
    init_info_map();
    init_planner();
    init_visualizer();

    this->use_plan_horizon = get_param<bool>(pnh, "use_plan_horizon", false);
    if (this->use_plan_horizon)
    {
        this->horizon_length = get_param<double>(pnh, "plan_horizon_length");
    }
}

void PlannerNodeMC::init_subscribers()
{
    // subscribes to new mission parameters
    // this->plan_request_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "plan_request_topic"), 1, &PlannerNodeMC::plan_request_callback, this);
    this->remaining_budget_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "remaining_budget_topic"), 1, &PlannerNodeMC::remaining_budget_callback, this);
    // this->agent_state_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "agent_odom_topic"), 1, &PlannerNodeMC::agent_state_callback, this);
    this->agent_state_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "agent_odom_topic"), 1, &PlannerNodeMC::agent_odom_callback, this);
    this->waypoint_num_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "waypoint_num_topic"), 1, &PlannerNodeMC::waypoint_reached_callback, this);
    // reatime search map metrics subscribers
    this->average_entropy_sub = nh.subscribe("realtime_search/average_entropy", 1, &PlannerNodeMC::average_entropy_callback, this);
    this->average_prob_sub = nh.subscribe("realtime_search/average_prob", 1, &PlannerNodeMC::average_prob_callback, this);
    //plan metrics subscriber
    this->plan_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "plan_output_topic"), 1, &PlannerNodeMC::plan_callback, this);
    //waypoint num subscriber
    this->waypoint_sub = nh.subscribe(ros_utils::get_param<std::string>(pnh, "waypoint_num_topic"), 1, &PlannerNodeMC::waypoint_num_callback, this);
    //target detections subscriber
    this->target_sub = nh.subscribe("sensor_measurement", 100, &PlannerNodeMC::target_detection_callback, this);
}

void PlannerNodeMC::init_publishers()
{
    this->final_path_pub = nh.advertise<planner_map_interfaces::Plan>(ros_utils::get_param<std::string>(pnh, "plan_output_topic"), 10);
    this->plan_request_pub = nh.advertise<planner_map_interfaces::PlanRequest>(ros_utils::get_param<std::string>(pnh, "plan_request_topic"), 1000);
    this->kill_all_nodes_pub = nh.advertise<planner_map_interfaces::PlanRequest>("/kill_all_nodes", 10);
}

void PlannerNodeMC::plan_request_callback(const planner_map_interfaces::PlanRequest &msg)
{
    this->latest_plan_req_msg = msg; // copy out the message. this is cheap because it's just built-in types

    ROS_INFO_STREAM("\n\n[PlannerNodeMC] Received Replan Message");
    ROS_INFO_STREAM("Max Planning Time: " << msg.max_planning_time);
    ROS_INFO_STREAM("Counter Detection Range: " << msg.counter_detection_range);
    ROS_INFO_STREAM("Maximum Agent Range: " << msg.maximum_range);
    ROS_INFO_STREAM("Desired Speed: " << msg.desired_speed);
    ROS_INFO_STREAM("Maximum Kappa: " << this->ipp_planner->max_kappa);

    this->current_plan_msg.plan.clear();
    this->current_plan_waypoint_map.clear();
    this->remaining_budget = msg.maximum_range;

    ROS_INFO_STREAM("debug start here");
    this->ipp_planner->save_plan_request_params(msg);
    ROS_INFO_STREAM("finish planner save param");
    this->info_map->save_plan_request_params(msg);
    this->info_map->send_plan_request_params_to_belief(msg);

    this->first_plan_request_received = true;
    this->has_new_plan_request = true;

    // publish planned runtime if we executing plan
    if (execute_plan)
    {   
        double max_budget = this->latest_plan_req_msg.maximum_range;
        double planned_run_time = max_budget / this->latest_plan_req_msg.desired_speed;
        ROS_WARN_STREAM("planned sim period is " << planned_run_time << " seconds");
    }

    // copy over yaml param files if they haven't already been copied
    std::string log_directory;
    nh.getParam("ipp_planners_node/plan_metrics_csv_directory", log_directory);
    //check if config directory
    // const fs::path config_path =; // Is this assignment safe?

    std::string config_dir =  log_directory + "/base_configs";

    if((this->log_plan_metrics || this->log_search_map_metrics) && !fs::exists(config_dir) && !fs::is_directory(config_dir))
    {
        ROS_INFO_STREAM("creating a base config dir at " << config_dir);
        // directory we save config to 
        std::string config_save_dir = log_directory + "base_configs";
        // get the config we're using
        std::string config = ros_utils::get_param<std::string>(pnh, "config");
        // copy configs from ipp_planners
        std::string ipp_planners_save_dir = config_save_dir + "/ipp_planners";
        std::string ipp_planners_dir = ros::package::getPath("ipp_planners") + "/config/" + config;
        fs::create_directories(ipp_planners_save_dir);
        fs::copy(ipp_planners_dir, ipp_planners_save_dir);
        // copy configs from ipp_simple sim
        std::string ipp_simple_sim_save_dir = config_save_dir + "/ipp_simple_sim";
        std::string ipp_simple_sim_dir = ros::package::getPath("ipp_simple_sim") + "/config/" + config;
        fs::create_directories(ipp_simple_sim_save_dir);
        fs::copy(ipp_simple_sim_dir, ipp_simple_sim_save_dir);
        // copy configs from planner_map_interfaces (ignore plan request subdirectories)
        std::string planner_map_interfaces_save_dir = config_save_dir + "/sensor_params";
        std::string planner_map_interfaces_dir = ros::package::getPath("planner_map_interfaces") + "/config/" + config;
        fs::create_directories(ipp_simple_sim_save_dir);
        fs::copy(planner_map_interfaces_dir, planner_map_interfaces_save_dir);
        // copy configs from simulated perception
        std::string simulated_perception_save_dir = config_save_dir + "/simulated_perception";
        std::string simulated_perception_dir = ros::package::getPath("simulated_perception") + "/config"; //one file in dir
        fs::create_directories(ipp_simple_sim_save_dir);
        fs::copy(simulated_perception_dir, simulated_perception_save_dir);
    }
}

// update the state of our agent
void PlannerNodeMC::remaining_budget_callback(const std_msgs::Float32 &msg)
{
    this->remaining_budget = msg.data;
}

// update the state of our agent
void PlannerNodeMC::agent_state_callback(const geometry_msgs::PoseStamped &msg)
{   
    // calculate the horizontal velocity of the agent
    // double dx = msg.pose.position.x - this->agent_pose.pose.position.x;
    // double dy = msg.pose.position.y - this->agent_pose.pose.position.y;
    // double dt = msg.header.stamp.toSec() - this->agent_pose.header.stamp.toSec();
    // double velocity = std::sqrt(dx * dx + dy * dy) / dt;
    // ROS_INFO_STREAM("Agent velocity: " << velocity);
    this->received_sim_startup_msg = true;
    this->agent_pose = msg;
    
    // ROS_INFO_STREAM("New agent pose at x::" << agent_pose.pose.position.x);
}

void PlannerNodeMC::agent_odom_callback(const nav_msgs::Odometry &msg)
{   
    // calculate the horizontal velocity of the agent
    // double dx = msg.pose.position.x - this->agent_pose.pose.position.x;
    // double dy = msg.pose.position.y - this->agent_pose.pose.position.y;
    // double dt = msg.header.stamp.toSec() - this->agent_pose.header.stamp.toSec();
    // double velocity = std::sqrt(dx * dx + dy * dy) / dt;
    // ROS_INFO_STREAM("Agent velocity: " << velocity);
    this->received_sim_startup_msg = true;
    
    this->agent_pose.header = msg.header;
    this->agent_pose.pose = msg.pose.pose;
    // ROS_INFO_STREAM("New agent pose at x::" << agent_pose.pose.position.x);
}

void PlannerNodeMC::average_entropy_callback(const std_msgs::Float32 &msg)
{
    this->average_entropy = msg.data;
}

void PlannerNodeMC::average_prob_callback(const std_msgs::Float32 &msg)
{
    this->average_prob = msg.data;
}

void PlannerNodeMC::target_detection_callback(const ipp_simple_sim::Detections &msg)
{
    if(sample_targets)
    {
        for(int idx = 0; idx < msg.target_ids.size(); ++idx)
        {
            //check to see if value exists inside the detected targets list
            auto iter = std::find(detected_target_ids.begin(), detected_target_ids.end(), msg.target_ids[idx]);
            if (iter == detected_target_ids.end())
            {
                ROS_WARN_STREAM("[PlannerNodeMC] Target " << msg.target_ids[idx] << " detected");
                detected_target_ids.push_back(msg.target_ids[idx]);
            }
        }
    }
}

void PlannerNodeMC::plan_callback(const planner_map_interfaces::Plan &msg)
{
    //ignore if not executing plan
    if(execute_plan && this->duration_between_replan > ros::Duration(0.0))
    {
        this->planner_log_file << std::fixed << ros::Time::now().toSec() <<
        "," << this->ipp_planner->get_plan_metrics() << std::endl;
    }
}

void PlannerNodeMC::waypoint_num_callback(const std_msgs::UInt32 &msg)
{
    //ignore if not executing plan
    if(execute_plan && !this->current_plan_msg.plan.empty())
    {
        //access waypoint at given waypoint index
        geometry_msgs::Pose waypoint_pose = this->current_plan_msg.plan[msg.data].position;
        //extract orientation from waypoint
        tf2::Quaternion orientation(waypoint_pose.orientation.x,
                                    waypoint_pose.orientation.y,
                                    waypoint_pose.orientation.z,
                                    waypoint_pose.orientation.w);
        tf2::Matrix3x3 orientation_mat(orientation);
        double roll, pitch, yaw;
        orientation_mat.getRPY(roll, pitch, yaw);
        // ROS_WARN_STREAM("Waypoint pose: " << waypoint_pose.position.x << " "
        // << waypoint_pose.position.y << " " << waypoint_pose.position.z << " " << yaw);
        //write pose to file
        this->plan_log_file << waypoint_pose.position.x << ","
                                    << waypoint_pose.position.y << ","
                                    << waypoint_pose.position.z << ","
                                    << yaw << std::endl;
    }
}

// subscribes to check what waypoint index has been reached
void PlannerNodeMC::waypoint_reached_callback(const std_msgs::UInt32 &msg)
{
    if (msg.data > this->waypoint_num && this->waypoint_num % 10 == 0)
    {
        ROS_INFO_STREAM("Waypoint " << (int)msg.data << " reached with budget " 
            << remaining_budget << " remaining");
    }
    this->waypoint_num = msg.data;
}

void PlannerNodeMC::propagate_info_map(double time_to_propagate)
{
    (void)time_to_propagate;
}

planner_map_interfaces::PlanRequest PlannerNodeMC::samplePlanRequest(std::vector<planner_map_interfaces::TargetPrior> target_priors)
{
    std::random_device rd;
    double seed_val = 0;
    if(pnh.hasParam("seed_1"))
    {
        std::vector<int> seeds = ros_utils::get_param<std::vector<int>>(pnh, "seed_1");
        if (loop_count < seeds.size()) {
            seed_val = seeds[loop_count];
        }
        else {
            ROS_WARN_STREAM("USING RANDOM SEED BECAUSE SEED ARRAY HAS BEEN EXHAUSTED");
            seed_val = rd();
        }
    }
    else 
    {
        seed_val = rd();
    }
    ROS_INFO_STREAM("Seed value for plan: " << seed_val);
    std::mt19937 gen(seed_val); 
    double map_bounds = ros_utils::get_param<double>(pnh, "map_bounds");

    std::uniform_real_distribution<> rand_range(ros_utils::get_param<double>(pnh, "min_budget"),
                                                ros_utils::get_param<double>(pnh, "max_budget"));
    std::uniform_real_distribution<> rand_xy_buffer(200,
                                                    map_bounds-200);
    std::uniform_real_distribution<> rand_yaw(-M_PI, M_PI);
    
    planner_map_interfaces::PlanRequest PlanRequest;
    

    PlanRequest.header.frame_id = local_frame;
    PlanRequest.header.stamp = ros::Time::now();
    PlanRequest.scenario = loop_count;

    // Choose random start pose
    PlanRequest.start_pose.position.x = rand_xy_buffer(gen);
    PlanRequest.start_pose.position.y = rand_xy_buffer(gen);
    PlanRequest.start_pose.position.z = this->flight_height;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, rand_yaw(gen));
    PlanRequest.start_pose.orientation.x = myQuaternion[0];
    PlanRequest.start_pose.orientation.y = myQuaternion[1];
    PlanRequest.start_pose.orientation.z = myQuaternion[2];
    PlanRequest.start_pose.orientation.w = myQuaternion[3];

    PlanRequest.current_speed.x = 0.0;
    PlanRequest.current_speed.y = 0.0;
    PlanRequest.current_speed.z = 0.0;

    PlanRequest.wind_speed.x = 0.0;
    PlanRequest.wind_speed.y = 0.0;
    PlanRequest.wind_speed.z = 0.0;
    PlanRequest.clear_tree = true;
    PlanRequest.max_planning_time = ros_utils::get_param<double>(pnh, "planning_time");

    PlanRequest.maximum_range = rand_range(gen);

    PlanRequest.desired_speed = ros_utils::get_param<double>(pnh, "drone_speed"); 

    geometry_msgs::Point32 bound1;
    bound1.x = 0;
    bound1.y = 0;
    bound1.z = 0;

    geometry_msgs::Point32 bound2;
    bound2.x = map_bounds+boundary_offset_for_search_map_reset;
    bound2.y = 0;
    bound2.z = 0;

    if(boundary_offset_for_search_map_reset)
    {
        boundary_offset_for_search_map_reset = 0;
    }
    else
    {
        boundary_offset_for_search_map_reset = 1;
    }

    geometry_msgs::Point32 bound3;
    bound3.x = map_bounds;
    bound3.y = map_bounds;
    bound3.z = 0;

    geometry_msgs::Point32 bound4;
    bound4.x = 0;
    bound4.y = map_bounds;
    bound4.z = 0;

    std::vector<geometry_msgs::Point32> vect{bound1, bound2, bound3, bound4};
    PlanRequest.search_bounds.points = vect;

    PlanRequest.counter_detection_range = 0;
    planner_map_interfaces::SensorConstraint sensorConstraints;
    PlanRequest.sensor_constraints = sensorConstraints;

    PlanRequest.target_priors = target_priors;

    return PlanRequest;
}

std::vector<planner_map_interfaces::TargetPrior> PlannerNodeMC::generateTargetPriors()
{
    double map_bounds = ros_utils::get_param<double>(pnh, "map_bounds");
    std::random_device rd;
    double seed_val = 0;
    if(pnh.hasParam("seed_2"))
    {
        std::vector<int> seeds = ros_utils::get_param<std::vector<int>>(pnh, "seed_2");
        if(loop_count < seeds.size()) {
            seed_val = seeds[loop_count];
        }
        else {
            ROS_WARN_STREAM("USING RANDOM SEED BECAUSE SEED ARRAY HAS BEEN EXHAUSTED");
            seed_val = rd();
        }
    }
    else
    {
        seed_val = rd();
    }
    ROS_INFO_STREAM("Seed value for targets: " << seed_val);
    std::mt19937 gen(seed_val); 
    std::uniform_int_distribution<> rand_num_targ(ros_utils::get_param<double>(pnh, "min_num_targets"),
                                                    ros_utils::get_param<double>(pnh, "max_num_targets"));
    std::uniform_real_distribution<> rand_targ_xy(0, map_bounds);
    std::uniform_real_distribution<> rand_yaw(-M_PI, M_PI);
    std::uniform_real_distribution<> rand_speed(ros_utils::get_param<double>(pnh, "min_target_speed"),
                                                ros_utils::get_param<double>(pnh, "max_target_speed"));
    std::uniform_real_distribution<> rand_xy_cov(ros_utils::get_param<double>(pnh, "min_target_cov"),
                                                    ros_utils::get_param<double>(pnh, "max_target_cov"));
    std::uniform_real_distribution<> rand_head_cov(ros_utils::get_param<double>(pnh, "min_head_cov"),
                                                    ros_utils::get_param<double>(pnh, "max_head_cov"));
    std::uniform_real_distribution<> rand_speed_cov(ros_utils::get_param<double>(pnh, "min_speed_cov"),
                                                    ros_utils::get_param<double>(pnh, "max_speed_cov"));

    std::vector<planner_map_interfaces::TargetPrior> target_priors;
    planner_map_interfaces::FilteredTarget filtered_target;
    boost::array<double, 16> cov_vector;
    planner_map_interfaces::TargetPrior target_prior;

    filtered_target.header.stamp = ros::Time::now();
    target_prior.header.stamp = ros::Time::now();

    int num_targets = rand_num_targ(gen);

    // Generate random targets
    for (int i = 1; i <= num_targets; i++)
    {
        filtered_target.global_id = i;
        double heading = rand_yaw(gen);
        double linear_speed = rand_speed(gen);
        filtered_target.xdot = linear_speed * cos(heading);
        filtered_target.ydot = linear_speed * sin(heading);

        // set some arbitrary values indicating a wide region but good with the velocity
        cov_vector.fill(0);
        double vel_cov = rand_xy_cov(gen);
        cov_vector[0] = vel_cov;  // x
        cov_vector[4] = vel_cov;  // y
        cov_vector[8] = rand_head_cov(gen); // heading
        cov_vector[12] = rand_speed_cov(gen);  // linear speed
        // cov_vector[24] = 1e-20; // angular speed
        filtered_target.covariance = cov_vector;

        filtered_target.x = rand_targ_xy(gen); 
        filtered_target.y = rand_targ_xy(gen);
        
        target_prior.target = filtered_target;
        target_priors.push_back(target_prior);
    }

    return target_priors;
}

void update_random_grid_prior_updates(std::unordered_map<std::string, 
                                      double>& grid_prior_updates, 
                                      int new_x, 
                                      int new_y, 
                                      double new_belief,
                                      double map_bounds)
{
    if (new_x >= 0 && new_x <= map_bounds && new_y >= 0 && new_y <= map_bounds)
    {
        std::string key = std::to_string(new_x) + "_" + std::to_string(new_y);
        std::unordered_map<std::string,double>::const_iterator itr = grid_prior_updates.find(key);
        if(itr==grid_prior_updates.end())
        {
            grid_prior_updates[key] = new_belief;
        }
        else
        {
            if (new_belief > itr->second)
                grid_prior_updates[key] = new_belief;
        }
    }
}

int roundUp(int numToRound, int multiple) 
{
    assert(multiple);
    return ((numToRound + multiple - 1) / multiple) * multiple;
}


std::vector<planner_map_interfaces::TargetPrior> PlannerNodeMC::generateSearchPriors()
{
    double map_bounds = ros_utils::get_param<double>(pnh, "map_bounds");
    std::random_device rd;
    double seed_val = 0;
    if(pnh.hasParam("seed_2"))
    {
        std::vector<int> seeds = ros_utils::get_param<std::vector<int>>(pnh, "seed_2");
        if(loop_count < seeds.size()) {
            seed_val = seeds[loop_count];
        }
        else {
            ROS_WARN_STREAM("USING RANDOM SEED BECAUSE SEED ARRAY HAS BEEN EXHAUSTED");
            seed_val = rd();
        }
    }
    else
    {
        seed_val = rd();
    }
    ROS_INFO_STREAM("Seed value for targets: " << seed_val);
    double map_resolution = ros_utils::get_param<double>(pnh, "map_resolution");
    std::mt19937 gen(seed_val); 
    double min_num_targets = ros_utils::get_param<double>(pnh, "min_num_targets");
    double max_num_targets = ros_utils::get_param<double>(pnh, "max_num_targets");
    double min_sigma = ros_utils::get_param<double>(pnh, "min_sigma");
    double max_sigma = ros_utils::get_param<double>(pnh, "max_sigma");
    ROS_WARN_STREAM("\n[PlannerNodeMC] Targets: " << min_num_targets << "-" << max_num_targets << " Sigma: " << min_sigma << "-" << max_sigma);

    std::uniform_int_distribution<> rand_num_targ(min_num_targets, max_num_targets);
    std::uniform_real_distribution<> rand_targ_xy(0, map_bounds);
    std::uniform_real_distribution<> rand_yaw(-M_PI, M_PI);
    std::uniform_real_distribution<> rand_sigma(min_sigma, max_sigma);
    std::uniform_real_distribution<> rand_confidence(ros_utils::get_param<double>(pnh, "min_confidence"),
                                                    ros_utils::get_param<double>(pnh, "max_confidence"));
    

    std::vector<planner_map_interfaces::TargetPrior> target_priors;
    planner_map_interfaces::GridPrior grid_prior;
    boost::array<double, 25> cov_vector;
    planner_map_interfaces::TargetPrior target_prior;

    grid_prior.header.stamp = ros::Time::now();
    target_prior.header.stamp = ros::Time::now();

    int num_targets = rand_num_targ(gen);

    geometry_msgs::Point32 point;
    double search_bounds_confidence = 0.05;

    std::unordered_map<std::string, double> grid_prior_updates;
    for (int i = 1; i <= num_targets; i++)
    {
        // Fill the geometry_msgs/Polygon message
        int x_center = roundUp(rand_targ_xy(gen), map_resolution);
        int y_center = roundUp(rand_targ_xy(gen), map_resolution);

        double sigma = rand_sigma(gen);
        double confidence = rand_confidence(gen);
        int window_step = 0;
        double scale_value = confidence/(1/(2*PI*sigma*sigma));

        while (search_bounds_confidence < (1/(2*PI*sigma*sigma))*exp(-((window_step)*(window_step))/(2*sigma*sigma))*scale_value) //just see if below initial confidence
        {
            // Update along outside edge of expanded window
            for (int i = -window_step; i <= window_step; i++)
            {
                double value = (1/(2*PI*sigma*sigma))*exp(-(i*i+window_step*window_step)/(2*sigma*sigma));
                double new_belief = value*scale_value;

                int new_x = x_center+i*map_resolution;
                int new_y = y_center+window_step*map_resolution;
                update_random_grid_prior_updates(grid_prior_updates, new_x, new_y, new_belief, map_bounds);

                new_x = x_center+i*map_resolution;
                new_y = y_center-window_step*map_resolution;
                update_random_grid_prior_updates(grid_prior_updates, new_x, new_y, new_belief, map_bounds);

            }
            for (int i = -std::max(0,window_step-1); i <= std::max(0,window_step-1); i++)
            {
                double value = (1/(2*PI*sigma*sigma))*exp(-(i*i+window_step*window_step)/(2*sigma*sigma));
                double new_belief = value*scale_value;

                int new_x = x_center+window_step*map_resolution;
                int new_y = y_center+i*map_resolution;
                update_random_grid_prior_updates(grid_prior_updates, new_x, new_y, new_belief, map_bounds);

                new_x = x_center-window_step*map_resolution;
                new_y = y_center+i*map_resolution;
                update_random_grid_prior_updates(grid_prior_updates, new_x, new_y, new_belief, map_bounds);
            }
            window_step++;
            // ROS_ERROR_STREAM("Last value: " << (1/(2*PI*sigma*sigma))*exp(-((window_step)*(window_step) + (window_step)*(window_step))/(2*sigma*sigma))*scale_value);
        }
    }

    //create a std::pair vector to do a weighted sample over for filtered targets
    std::vector<double> prior_weights;

    grid_prior.grid_prior_type = planner_map_interfaces::GridPrior::POINT_PRIOR;
    grid_prior.confidence = 0.5;
    grid_prior.priority = 1.0;
    grid_prior.sensor_model_id = 0;
    // Fill the geometry_msgs/Polygon message
    // Loop over grid_prior_updates and make a grid prior for each
    for (auto const& x : grid_prior_updates)
    {
        // ROS_ERROR_STREAM("Key: " << x.first << " Value: " << x.second);
        std::string key = x.first;
        double value = x.second;
        std::string delimiter = "_";
        size_t pos = 0;
        std::string token;
        std::vector<double> prior;
        while ((pos = key.find(delimiter)) != std::string::npos) {
            token = key.substr(0, pos);
            prior.push_back(std::stod(token));
            key.erase(0, pos + delimiter.length());
        }
        prior.push_back(std::stod(key));
        // ROS_ERROR_STREAM("X: " << prior[0] << " Y: " << prior[1] << " Value: " << prior[2]);
        geometry_msgs::Point32 point;
        point.x = prior[0];
        point.y = prior[1];
        grid_prior.bounds.points.push_back(point);
        grid_prior.confidence = value;

        target_prior.grid_prior = grid_prior;
        target_priors.push_back(target_prior);
        prior_weights.push_back(grid_prior.confidence);
        grid_prior.bounds.points.clear();
    }

    if(sample_targets)
    {
        ROS_WARN_STREAM("[PlannerNodeMC] Sampling " << num_targets << " target(s) from prior");
        std::default_random_engine generator;
        target_prior.grid_prior = planner_map_interfaces::GridPrior(); //clear existing grid prior
        std::discrete_distribution<int> distribution(prior_weights.begin(), prior_weights.end());

        for(int i = 1; i <= num_targets; i++) //target ID always set to > 0. Important for logging loop
        {
            int rand_idx = distribution(generator);
            planner_map_interfaces::GridPrior rand_grid_prior = target_priors[rand_idx].grid_prior;

            //populate a filtered target message for this sampled grid prior
            planner_map_interfaces::FilteredTarget sampled_target;
            sampled_target.x = rand_grid_prior.bounds.points[0].x;
            sampled_target.y = rand_grid_prior.bounds.points[0].y;
            sampled_target.xdot = 0.0;
            sampled_target.ydot = 0.0;
            sampled_target.global_id = i;
            sampled_target.local_id = i;
            target_prior.target = sampled_target;
            target_priors.push_back(target_prior);
        }
    }

    //log number of targets to a directory
    if(sample_targets)
    {
        //add ground truth for all planner types
        for(int i = 0; i < planner_types.size(); ++i)
        {
            this->gt_target_log_file << planner_types[i] << 
            "," << loop_count + 1 << "," << num_targets << std::endl;
        }
    }

    return target_priors;
}

bool PlannerNodeMC::all_targets_outside_of_bounds()
{
    return false;
} 
