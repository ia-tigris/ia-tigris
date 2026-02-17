#pragma once

#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <random>
#include <chrono>
#include <unordered_map>

#include "planner_map_interfaces/ros_utils.h"

#include "ipp_planners/InfoMapTrack.h"
#include "ipp_planners/CoveragePlanner.h"
#include "ipp_belief/belief_manager.h"
#include "ipp_belief/information.h"
#include "ipp_belief/trackers.h"

#include "unistd.h"
namespace ipp
{
    class TopoNode : public TreeNode
    {
    protected:
        std::unique_ptr<InfoMapTrack> info_map;
        std::unique_ptr<InfoMapTrack> info_map_before_cov;
        int node_level;
        bool expandable; // path still within budget, and not denied with branch & bound
        bool fully_expanded;
        std::vector<unsigned int> target_ids;
        size_t target_to_expand_idx;
        unsigned int curr_target_id;
        unsigned int target_to_expand; // next target to expand, set to 0 (first target) when initialized
        std::vector<double> node_start_pose;
    public:
        // debug tools here, clear after debugger
        std::string special_note;

        // debug tools end here
        TopoNode* parent_topo;
        double rollout_variance;   // estimated value of the future
        double var_after_cover;
        std::vector<TopoNode> child_nodes;
        std::vector<std::vector<double>> coverage_path;
        std::unordered_map<unsigned int, double> last_observed_time;
        std::vector<std::vector<double>> target_coverage_boundary;

        double get_value()
        {
            return rollout_variance;
        }

        TopoNode(const ob::SpaceInformationPtr &si)
            : TreeNode(si),
              fully_expanded(false),
              expandable(true),
              node_level(-1),
              target_to_expand_idx(0),
              curr_target_id(0),
              rollout_variance(-1),
              parent_topo(nullptr){}

        virtual ~TopoNode() {}

        TopoNode *empty_clone() override
        {
            if (!this->info_map)
            {
                throw std::runtime_error("info_map is null");
            }
            auto node = new TopoNode(this->si_);
            return node;
        }

        void set_curr_target_id(size_t target_id)
        {
            curr_target_id = target_id;
        }

        unsigned int get_curr_target_id()
        {
            return curr_target_id;
        }

        void set_node_start_pose(std::vector<double> input_start_pose)
        {
            node_start_pose = input_start_pose;
        }

        std::vector<double> get_node_start_pose()
        {
            return node_start_pose;
        }

        void init_target_to_expand()
        {
            target_to_expand_idx = 0;
            target_to_expand = target_ids[target_to_expand_idx];
        }

        void set_next_target_to_expand()
        {
            target_to_expand_idx++;
            if (target_to_expand_idx >= target_ids.size())
            {
                fully_expanded = true;
            }
            else
            {
                target_to_expand = target_ids[target_to_expand_idx];
            }
        }

        void set_target_ids(std::vector<unsigned int> target_ids_)
        {
            target_ids = target_ids_;
        }

        unsigned int get_target_to_expand()
        {
            return target_to_expand;
        }

        void set_level(int level)
        {
            node_level = level;
        }

        int get_level()
        {
            return node_level;
        }

        void become_unexpandable()
        {
            expandable = false;
        }

        bool check_fully_expand()
        {
            return fully_expanded;
        }

        bool check_expandable()
        {
            return expandable;
        }

        void add_child(TopoNode* child)
        {
            children.push_back(child);
        }

        void copy_info_map_params_from(TopoNode &other)
        {
            this->info_map->copy_params_from(*(other.get_info_map()));
        }

        void reset_sampling_state()
        {
            if (info_map)
                info_map->reset_sampling_state();
            else
                ROS_WARN("Asked to reset_sampling_state but no info map to reset resampling state");
        }

        // get info map. uses getters to warn in case nullptr
        InfoMapTrack* get_info_map()
        {
            if (!info_map)
                throw std::runtime_error("Asked to get_info_map but no info map to return");
            return info_map.get();
        }

        void set_info_map(std::unique_ptr<InfoMapTrack> info_map)
        {
            this->info_map = std::move(info_map);
        }

        InfoMapTrack* get_info_map_before_cov()
        {
            if (!info_map_before_cov)
                throw std::runtime_error("Asked to get_info_map but no info map to return");
            return info_map_before_cov.get();
        }

        void set_info_map_before_cov(std::unique_ptr<InfoMapTrack> info_map)
        {
            this->info_map_before_cov = std::move(info_map);
        }

        void set_belief(tracking::ParticleFiltersBelief &belief)
        {
            if (info_map)
                info_map->set_particle_belief_manager(belief);
            else
                throw std::runtime_error("Asked to set_belief but no info map to set belief");
        }
    };

    class TopoTree
    {
    protected:
        const ompl::base::SpaceInformationPtr XYZPsi_Space;
        TopoNode* tree_root;
        TopoNode* leaf_node_of_best_path;
        std::vector<unsigned int> target_ids;
        bool include_edge = false;
        bool best_path_updated = false;
        double total_budget;
        double desired_speed;
        std::queue<TopoNode*> nodes_to_expand;
        std::vector<TopoNode*> all_nodes;
        double min_avg_var;
        double min_rollout_var;

    public:
        TopoTree(const ompl::base::SpaceInformationPtr XYZPsi_Space_):
            XYZPsi_Space(XYZPsi_Space_) {}
        
        ~TopoTree()
        {
            for (auto it : all_nodes)
            {
                delete it;
            }
        }
        
        bool has_best_path_updated()
        {
            return best_path_updated;
        }

        std::queue<TopoNode*> get_expand_list()
        {
            return  nodes_to_expand;
        }

        void initialize(InfoMap &info_map_, std::vector<double> start_pose, double budget, double desired_speed_)
        {
            ob::ScopedState<XYZPsiStateSpace> start_n(XYZPsi_Space);
            Eigen::Vector3d start_v(start_pose[0], start_pose[1], start_pose[2]);
            start_n->setXYZ(start_v);
            start_n->setPsi(start_pose[3]);
            XYZPsi_Space->enforceBounds(start_n.get());

            auto* node = new TopoNode(XYZPsi_Space);
            XYZPsi_Space->getStateSpace()->copyState(node->state, start_n.get());
            node->set_node_start_pose(start_pose);
            node->budget_remaining = budget;
            tree_root = node;

            total_budget = budget;
            desired_speed = desired_speed_;
            
            tree_root->information = info_map_.calc_child_to_root_value(*node, include_edge); // From tigris, not sure if still needed
            leaf_node_of_best_path = node; // Set starting node as best path.

            auto info_map_track = dynamic_cast<InfoMapTrack &>(info_map_);
            auto belief_manager = info_map_track.get_particle_belief_manager();
            
            target_ids = belief_manager.get_tracker_ids();
            
            // Avg var (value/score) at the beginning
            min_avg_var = tracking::calculate_belief_variance(belief_manager) / (target_ids.size());
            min_rollout_var = -1;
            // ROS_INFO_STREAM("starting with avg cov: " << min_avg_var);
            for (auto& [id, tracker] : belief_manager.id_to_trackers)
            {
                tree_root->last_observed_time.insert({id, tracker.get_last_observed_time()});
            }
            tree_root->set_target_ids(target_ids);
            tree_root->init_target_to_expand();
            tree_root->set_info_map(info_map_track.clone());
            tree_root->set_info_map_before_cov(info_map_track.clone());
            tree_root->set_level(0);
            tree_root->var_after_cover = min_avg_var;
            for (size_t i = 0; i < target_ids.size(); i++)
            {
                // expand n times to cover all possible first targets
                nodes_to_expand.push(tree_root);   
            }
            all_nodes.push_back(tree_root);
        }

        // branch and bound
        void solve_bb()
        {
            
        }

        void expand_tree()
        {
            if (nodes_to_expand.empty())
            {
                // ROS_INFO_STREAM("expand list empty");
                return;
            }
            expand_node(nodes_to_expand.front());
            nodes_to_expand.pop();
        }

        std::vector<TopoNode*> get_nodes()
        {
            return all_nodes;
        }

        void expand_node(TopoNode* node)
        {
            // check if it is expandable (not denied) or fully expanded
            if (!node->check_expandable() || node->check_fully_expand())
            {
                std::string condition_text;
                // debug message
                (!node->check_expandable()) ? condition_text = "non expandable" : condition_text = "fully expanded";
                if (!node->check_expandable() && node->check_fully_expand())
                {
                    condition_text = "non expandable and fully expanded";
                }
                ROS_WARN_STREAM("trying to expand a " << condition_text << " node, debug needed");
                return;
            }

            if (node->get_curr_target_id() == node->get_target_to_expand())
            {
                node->set_next_target_to_expand();
                return;
            }

            best_path_updated = false;

            // ROS_INFO_STREAM("expanding node at level " << node->get_level() << " with next target id " << node->get_target_to_expand());

            std::vector<double> start_pose = node->get_node_start_pose();

            double plan_height = start_pose[2];

            InfoMapTrack* info_map = node->get_info_map();
            auto old_particle_belief_manager = info_map->get_particle_belief_manager();
            tracking::ParticleFilter target_tracker = old_particle_belief_manager.get_tracker(node->get_target_to_expand());
            tracking::TargetState cluster_mean_state = target_tracker.get_mean_particle();

            TargetCentroid target_centroid = {cluster_mean_state.get_x(),
                                              cluster_mean_state.get_y(),
                                              cluster_mean_state.get_speed(),
                                              cluster_mean_state.get_heading(),
                                              tracking::calculate_trace_variance(target_tracker)};

            std::pair<double, double> est_intersect = solve_soonest_intersection_drone_to_target(start_pose[0], start_pose[1], desired_speed, target_centroid);
            
            // reaching time
            double est_reach_time = std::sqrt(std::pow(est_intersect.first - start_pose[0], 2) + std::pow(est_intersect.second - start_pose[1], 2)) / desired_speed;
            
            
            TopoNode* child_node = new TopoNode(XYZPsi_Space);
            child_node->set_curr_target_id(node->get_target_to_expand());
            child_node->set_target_ids(target_ids);
            child_node->init_target_to_expand();
            child_node->set_level(node->get_level() + 1);
            
            // set new infomap, which is the est future state after reaching AND covering the target
            std::unique_ptr<InfoMapTrack> new_info_map = std::make_unique<InfoMapTrack>(*info_map);
            
            // est particles after reach, before coverage
            new_info_map->particle_belief_manager.hyper_propagate(est_reach_time);

            // Roughly estimate coverage time according to propagated target particles
            tracking::ParticleFilter& target_tracker_after_reach = new_info_map->particle_belief_manager.get_editable_tracker(node->get_target_to_expand());
            
            std::vector<std::vector<double>> rectangle_corners = get_boundary_corners(new_info_map->particle_belief_manager.get_tracker(node->get_target_to_expand()).get_particles(), plan_height);
            child_node->target_coverage_boundary = rectangle_corners;
            double est_cover_time = 0;
            RectangleCoverage cov_plan({0., 0., plan_height}, 100., 50.);
            double est_cover_len = cov_plan.get_rect_coverage_length(rectangle_corners);
            child_node->coverage_path = cov_plan.rect_coverage(rectangle_corners);
            est_cover_time = est_cover_len / desired_speed;
            child_node->set_info_map_before_cov(new_info_map->clone());
            // est particles after coverage, save as start status for the child node
            new_info_map->particle_belief_manager.hyper_propagate(est_cover_time);
            tracking::TargetState after_reach_mean_state = target_tracker_after_reach.get_mean_particle();
            tracking::TargetState mean_state_after_reach (node->get_target_to_expand(), after_reach_mean_state.get_x(), after_reach_mean_state.get_y(),
                                                            after_reach_mean_state.get_z(), after_reach_mean_state.get_heading(),
                                                            after_reach_mean_state.get_speed(), 0,
                                                            1.0);
            // tracking::TargetGaussian dis_after_reach(mean_state_after_reach); // uncertainty here needs discussion
            tracking::TargetGaussian dis_after_reach = tracking::make_target_gaussian_from_fake_observed_target(mean_state_after_reach);
            target_tracker_after_reach.init_particles(dis_after_reach); // collapse target to mean state with some uncertainty

            double remain_budget = node->budget_remaining - est_reach_time * desired_speed - est_cover_len;
            child_node->budget_remaining = remain_budget;
            if (child_node->budget_remaining < 0)
            {
                child_node->become_unexpandable();
            }
            else
            {
                for (size_t i = 0; i < target_ids.size(); i++)
                {
                    nodes_to_expand.push(child_node);
                }
            }
            
            // set child node start pose
            std::vector<double> new_start_pose = {after_reach_mean_state.get_x(), after_reach_mean_state.get_y(), start_pose[2], after_reach_mean_state.get_heading()};
            child_node->set_node_start_pose(new_start_pose);

            child_node->parent = node;
            child_node->parent_topo = node;
            child_node->set_info_map(new_info_map->clone());
            node->add_child(child_node);
            node->set_next_target_to_expand();
            
            // check best path or not
            double curr_avg_var = tracking::calculate_belief_variance(new_info_map->particle_belief_manager) / target_ids.size();
            child_node->var_after_cover = curr_avg_var;
            child_node->last_observed_time = node->last_observed_time;
            child_node->last_observed_time.at(child_node->get_curr_target_id()) += (total_budget - child_node->budget_remaining) / desired_speed;
            
            // double curr_var_time = curr_avg_var / (total_budget - child_node->budget_remaining);
            
            // minimum avg var / total budget cost
            // if (curr_var_time < min_avg_var && child_node->budget_remaining >= 0)
            // {
            //     min_avg_var = curr_var_time;
            //     leaf_node_of_best_path = child_node;
            //     best_path_updated = true;
            // }

            // minimum sum (avg var) * each budget cost, consider like an integral

            // double curr_value = weighted_int_var_in_path(child_node, total_budget);
            double curr_value = get_rollout_value(child_node, total_budget);

            if (min_rollout_var < 0)
            {
                min_rollout_var = curr_value;
                leaf_node_of_best_path = child_node;
                best_path_updated = true;
            }
            else if (curr_value < min_rollout_var && child_node->budget_remaining >= 0)
            {
                min_rollout_var = curr_value;
                leaf_node_of_best_path = child_node;
                best_path_updated = true;
            }

            

            if (child_node->budget_remaining >= 0)
            {
                // get current path
                ROS_INFO_STREAM("current path is:");
                print_path_from_node(child_node);

                ROS_INFO_STREAM("current path value is:");
                std::cout << curr_value << '\n';
                print_rollout_values(child_node, total_budget);

                ROS_INFO_STREAM("best path is:");
                print_path_from_node(leaf_node_of_best_path);

                ROS_INFO_STREAM("best path value is:");
                std::cout << min_rollout_var << '\n';

                print_rollout_values(leaf_node_of_best_path, total_budget);

                ROS_INFO_STREAM("and the budget so far in the planner is: " << remain_budget);
            }
            // ROS_INFO_STREAM("estimated coverage length in the latest node is: " << est_cover_len);

            // ROS_INFO_STREAM("best path is:");
            // print_path_from_node(leaf_node_of_best_path);

            // ROS_INFO_STREAM("best path value is:");
            // std::cout << min_rollout_var << '\n';

            // print_rollout_values(leaf_node_of_best_path, total_budget);

            // ROS_INFO_STREAM("and the budget so far in the planner is: " << remain_budget);

            // ROS_INFO_STREAM("avg var so far is: " << curr_avg_var);

            // Check the variance of each target tracker
            // std::vector<double> variance_list = tracking::calculate_variance_per_tracker(new_info_map->particle_belief_manager);
            // // ROS_INFO_STREAM("variance for targets are: ");
            // for (auto& n : variance_list)
            // {
            //     std::cout << n << ", ";
            // }
            // std::cout << "\n";
        }
        
        // variance upper bound rollout, assuming the drone doesn't expand in the future and stick to the target of the last node 
        double get_rollout_value(TopoNode* node, double total_budget)
        {
            double time_till_end = node->budget_remaining / desired_speed;
            if (time_till_end < 0)
            {
                return INFINITY;
            }
            double sum_variance = 0;
            double sum_std_dev_rollout = 0;
            std::map<unsigned int, double> id_var = tracking::calculate_xy_variance_per_tracker_map(node->get_info_map()->particle_belief_manager);
            std::map<unsigned int, double> id_spd = tracking::calculate_avg_speed_per_tracker_map(node->get_info_map()->particle_belief_manager);
            
            InfoMapTrack* info_map = node->get_info_map();
            std::unique_ptr<InfoMapTrack> rollout_info_map = std::make_unique<InfoMapTrack>(*info_map);

            rollout_info_map->particle_belief_manager.hyper_propagate(time_till_end);
            std::map<unsigned int, double> id_var_rollout = tracking::calculate_xy_variance_per_tracker_map(rollout_info_map->particle_belief_manager);

            for (auto& [id, var] : id_var)
            {
                if (id == node->get_curr_target_id())
                {
                    double std_dev_target = shift_leaky_relu(std::sqrt(var));
                    sum_std_dev_rollout += std_dev_target;
                    continue;
                }
                // double var_rollout = std::pow(std::sqrt(var) + id_spd.at(id) * time_till_end, 2);
                // sum_variance += sigmoid_trans(var_rollout);
                double std_dev_rollout = std::sqrt(id_var_rollout.at(id));
                sum_std_dev_rollout += shift_leaky_relu(std_dev_rollout);
                // ROS_INFO_STREAM("get rollout: target " << id << " was last detected at time" << node->last_observed_time.at(id));
            }
            
            return sum_std_dev_rollout;
        }

        void print_rollout_values(TopoNode* node, double total_budget)
        {
            double time_till_end = node->budget_remaining / desired_speed;
            ROS_INFO_STREAM("path rollout time is " << time_till_end);

            std::map<unsigned int, double> id_var = tracking::calculate_xy_variance_per_tracker_map(node->get_info_map()->particle_belief_manager);
            std::map<unsigned int, double> id_spd = tracking::calculate_avg_speed_per_tracker_map(node->get_info_map()->particle_belief_manager);

            InfoMapTrack* info_map = node->get_info_map();
            std::unique_ptr<InfoMapTrack> rollout_info_map = std::make_unique<InfoMapTrack>(*info_map);

            rollout_info_map->particle_belief_manager.hyper_propagate(time_till_end);
            std::map<unsigned int, double> id_var_rollout = tracking::calculate_xy_variance_per_tracker_map(rollout_info_map->particle_belief_manager);

            for (auto& [id, var] : id_var)
            {
                if (id == node->get_curr_target_id())
                {
                    double std_dev_target = shift_leaky_relu(std::sqrt(var));
                    ROS_INFO_STREAM("target id " << id << " value is " << std_dev_target);
                    continue;
                }
                // double var_rollout = std::pow(std::sqrt(var) + id_spd.at(id) * time_till_end, 2);
                // sum_variance += sigmoid_trans(var_rollout);
                double std_dev_rollout = std::sqrt(id_var_rollout.at(id));
                ROS_INFO_STREAM("target id " << id << " before rollout var is " << std::sqrt(var));
                // ROS_INFO_STREAM("target id " << id << " speed is " << id_spd.at(id));
                ROS_INFO_STREAM("target id " << id << " rollout value is " << shift_leaky_relu(std_dev_rollout));
                // ROS_INFO_STREAM("get rollout: target " << id << " was last detected at time" << node->last_observed_time.at(id));
            }
        }

        // double
        double shift_leaky_relu(double v_in)
        {
            double thres = 16000;
            if (v_in < 0)
            {
                return 0.0;
            }
            else if (v_in < thres)
            {
                return v_in;
            }
            
            return thres + 2 * (v_in - thres);
            
        }

        double sigmoid_trans(double v_in)
        {
            // param v_thres is the threshold of variance over which we consider the tracker is not quite valid, use as an offset in a sigmoid
            // param k_v is a coefficient in the sigmoid to control how quick the value grows
            // ROS_INFO_STREAM("in value is " << v_in);
            double v_thres = 100000;
            double k_v = 0.0001;
            return 1 / (1 + std::exp(- k_v * (v_in - v_thres)));
        }

        static std::pair<double, double> solve_soonest_intersection_drone_to_target(double drone_x, double drone_y, double drone_speed, TargetCentroid &target_pose)
        {
            double P0x = target_pose.x;
            double P0y = target_pose.y;
            double s0 = target_pose.speed;
            double V0x = std::cos(target_pose.heading);
            double V0y = std::sin(target_pose.heading);
            double P1x = drone_x, P1y = drone_y;
            double s1 = drone_speed;

            // quadratic formula
            double a = (s0 * s0) * ((V0x * V0x) + (V0y * V0y)) - (s1 * s1);
            double b = 2 * s0 * ((P0x * V0x) + (P0y * V0y) - (P1x * V0x) - (P1y * V0y));
            double c = (P0x * P0x) + (P0y * P0y) + (P1x * P1x) + (P1y * P1y) - (2 * P1x * P0x) - (2 * P1y * P0y);

            double t = NAN;
            if (a == 0)
            {
                t = -c / b;
            }
            else
            {
                double t1 = (-b + std::sqrt((b * b) - (4 * a * c))) / (2 * a);
                double t2 = (-b - std::sqrt((b * b) - (4 * a * c))) / (2 * a);

                t = choose_best_time(t1, t2);
            }

            if (isnan(t))
            {
                ROS_WARN_STREAM(
                    "No solution for intersection: "
                    << "drone_x=" << drone_x << " drone_y=" << drone_y << " drone_speed=" << drone_speed
                    << " paricle x= " << target_pose.x << " particle y= " << target_pose.y
                    << " particle speed= " << target_pose.speed << " particle heading= " << target_pose.heading);
                return std::make_pair(t, t); // returns nans
            }

            double intersect_x = P0x + t * s0 * V0x;
            double intersect_y = P0y + t * s0 * V0y;
            return std::make_pair(intersect_x, intersect_y);
        }

        static double choose_best_time(double t1, double t2)
        {
            auto positive_and_not_nan = [](double t)
            { return t > 0 && !std::isnan(t); };

            if (positive_and_not_nan(t1) && positive_and_not_nan(t2))
            {
                return std::min(t1, t2);
            }
            else if (positive_and_not_nan(t1))
            {
                return t1;
            }
            else if (positive_and_not_nan(t2))
            {
                return t2;
            }
            else
            {
                return std::nan("no solution");
            }
        }
        
        std::list<TopoNode*> trace_path_from_node(TopoNode* end_node)
        {
            if (end_node->parent_topo == nullptr)
            {
                return {end_node};
            }

            TopoNode* curr_node = end_node;
            std::list<TopoNode*> path_targets;
            while (curr_node->parent_topo != nullptr)
            {
                path_targets.push_front(curr_node);
                curr_node = curr_node->parent_topo;
            }
            path_targets.push_front(curr_node);
            return path_targets;
        }

        std::list<TopoNode*> get_best_path()
        {
            return trace_path_from_node(leaf_node_of_best_path);
        }

        TopoNode* get_best_path_end_node()
        {
            return leaf_node_of_best_path;
        }

        std::list<unsigned int> trace_path_id_from_node(TopoNode* end_node)
        {
            if (end_node->parent_topo == nullptr)
            {
                return {0};
            }

            TopoNode* curr_node = end_node;
            std::list<unsigned int> path_targets;
            while (curr_node->parent_topo != nullptr)
            {
                path_targets.push_front(curr_node->get_curr_target_id());
                curr_node = curr_node->parent_topo;
            }
            path_targets.push_front(curr_node->get_curr_target_id());
            return path_targets;
        }

        void print_path_from_node(TopoNode* end_node)
        {
            int path_length = 0;
            std::list<unsigned int> trace_path_from_child = trace_path_id_from_node(end_node);
            for (auto& n : trace_path_from_child)
            {
                std::cout << n << "->";
                path_length += 1;
            }
            std::cout << "\n";
            // if (path_length >= 2)
            // {
            //     usleep(1000000);
            // }
        }

        double weighted_int_var_in_path(TopoNode* end_node, double total_budget)
        {
            if (end_node->parent_topo == nullptr)
            {
                return -1;
            }
            
            double total_budget_cost = total_budget - end_node->budget_remaining;
            TopoNode* curr_node = end_node;
            double path_value = 0; // want the sum to be small
            while(curr_node->parent_topo != nullptr)
            {
                double curr_var = curr_node->var_after_cover;
                double last_budget_cost = curr_node->parent_topo->budget_remaining - curr_node->budget_remaining;
                path_value += curr_var * last_budget_cost / total_budget_cost;
                curr_node = curr_node->parent_topo;
            }

            return path_value;
        }

        std::vector<std::vector<std::vector<double>>> get_path_cov_bound()
        {
            std::vector<std::vector<std::vector<double>>> cover_bound_list;
            TopoNode* curr_node = leaf_node_of_best_path;
            while (curr_node->parent_topo != nullptr)
            {
                cover_bound_list.push_back(curr_node->target_coverage_boundary);
                curr_node = curr_node->parent_topo;
            }
            return cover_bound_list;
        }

        std::vector<std::vector<double>> get_boundary_corners(const std::vector<tracking::TargetState>& particles, double height)
        {
            // simplest method, max and min of x and y
            if (particles.empty())
            {
                ROS_WARN_STREAM("no particles, cannot get boundary");
                return {{0., 0.}, {0., 0.}, {0., 0.}, {0., 0.}};
            }

            double min_x = particles[0].get_x();
            double max_x = particles[0].get_x();
            double min_y = particles[0].get_y();
            double max_y = particles[0].get_y();

            for (auto& p : particles)
            {
                if (p.get_x() < min_x)
                {
                    min_x = p.get_x();
                }
                if (p.get_x() > max_x)
                {
                    max_x = p.get_x();
                }
                if (p.get_y() < min_y)
                {
                    min_y = p.get_y();
                }
                if (p.get_y() > max_y)
                {
                    max_y = p.get_y();
                }
            }
            return {{max_x, max_y, height}, {min_x, max_y, height}, {min_x, min_y, height}, {max_x, min_y, height}};
        }
    };
}