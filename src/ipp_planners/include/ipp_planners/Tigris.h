#pragma once

#include <eigen3/Eigen/Dense>
#include <tf/tf.h>
#include <fstream>
#include <optional>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include "ompl/tools/config/SelfConfig.h"

#include "ipp_planners/Planner.h"
#include "ipp_planners/SearchMap.h"
#include "ipp_planners/TreeNode.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;

#define NUM_DOF 4
#define MIN_ALT 10
#define ALT_STEP_SIZE 10


#define OURS 0
#define BENCHMARK_ONE 1

namespace ipp
{
    class Tigris : public Planner
    {

        int num_samples_drawn = 0;
    protected:
        // andrew added this to debug. TODO: move to visualization
        std::vector<std::vector<double>> xyzi_samples;

        std::random_device rd;
        std::mt19937 gen;
        std::discrete_distribution<> weighted_sampler;
        std::discrete_distribution<> distribution_z;
        std::uniform_real_distribution<> distribution_psi;

        /* TREE INFORMATION */
        TreeNode *tree_root;           // where the drone starts
        TreeNode *leaf_node_of_best_path; // ending node of the best path. traverse up parents to find the full path
        
        // A nearest-neighbors datastructure containing the tree of nodes.
        // Has to contain pointers, because we want to modify the data internally
        ompl::NearestNeighborsGNAT<TreeNode *> nn_data_structure;

    public:
        // tree
        double extend_dist;
        double extend_radius;
        double prune_radius;

        // ratio version of tree params, if set by user
        double extend_dist_ratio;
        double extend_radius_ratio;
        double prune_radius_ratio;

        Tigris(ros::NodeHandle &nh, ros::NodeHandle &pnh);

        ~Tigris();

        /* ====== OVERRIDDEN METHODS ====== */
        /**
         * @brief Set the params from plan request msg object. May be overriden. Make sure to call superclass version.
         *
         * @param msg
         */
        virtual bool save_plan_request_params(const planner_map_interfaces::PlanRequest &msg)
        {
            if (msg.planner_params.use_gimbal)
                this->sensor_params = fetch_sensor_params_from_rosparam_server_for_gimbal_planner(this->nh);
            bool clear_trees = Planner::save_plan_request_params(msg);
            if (use_ratio_distance_params)
            {
                ROS_ERROR_STREAM("Using ratio distance params is not implemented yet");
                throw std::runtime_error("Using ratio distance params is not implemented yet");
                // double max_edge_size = this->info_map->get_max_edge_size();
                // this->ipp_planner->extend_dist = max_edge_size * extend_dist_ratio;
                // this->ipp_planner->extend_radius = max_edge_size * extend_radius_ratio;
                // this->ipp_planner->prune_radius = max_edge_size * prune_radius_ratio;
                // ROS_INFO_STREAM("Update proportional tree parameters: " << this->ipp_planner->extend_dist << " " << this->ipp_planner->extend_radius << " " << this->ipp_planner->prune_radius);
            }
            return clear_trees;
        }

        virtual bool replan_setup(
            InfoMap &info_map,
            std::vector<double> start_pose,
            double budget, bool should_clear_tree) override;
        virtual bool replan_loop(
            InfoMap &info_map,
            std::vector<double> start_pose,
            double budget, bool should_clear_tree) override;
        virtual bool replan_teardown(
            InfoMap &info_map,
            std::vector<double> start_pose,
            double budget, bool should_clear_tree) override;
        
        virtual std::string get_plan_metrics() override;
        
        std::vector<og::PathGeometric> get_best_path_segments() override
        {
            std::vector<og::PathGeometric> segments;

            auto node_ptr = this->get_leaf_node_of_best_path();

            // trace from the leaf (end) to the root (beginning), then reverse
            while (node_ptr->parent != nullptr)
            {
                // performs a local copy
                segments.push_back(node_ptr->edge_trochoid);
                node_ptr = node_ptr->parent;
            }
            std::reverse(segments.begin(), segments.end());

            return segments;
        }

    private:
        /* ====== HELPER METHODS ====== */
        void update_tree(std::vector<double> &start_pose, InfoMap &info_map, bool should_clear_tree = false);
        void reset_tree_to_pose(std::vector<double> &start_pose, InfoMap &info_map);
        TreeNode *find_closest_tree_node_to_pose(std::vector<double> &start_pose);
        void recycle_tree_from_node(TreeNode *node_ptr, InfoMap &info_map);
        void erase_nodes_before_start(TreeNode *node_ptr);
        void erase_node_and_children(TreeNode *node_ptr);
        TreeNode *sample_node_within_budget(InfoMap &info_map, double budget);
        TreeNode sample_node(InfoMap &info_map);
        bool add_node_to_tree_or_prune(InfoMap &info_map, TreeNode *add_node, TreeNode *add_node_parent);
        void extend_neighbors_towards_node(InfoMap &info_map, TreeNode *node_feasible, double budget);
        bool is_valid(const ompl::base::State *state);
        bool should_prune(TreeNode *new_node, double radius) const;
        void add_children_to_tree(InfoMap &info_map, TreeNode *parent);
        void delete_node_and_children(TreeNode *parent);

        /* ==== GETTERS AND SETTERS ==== */
    public:
        const std::vector<TreeNode *> get_all_nodes() const;
        TreeNode *get_leaf_node_of_best_path() const;
        const std::vector<std::vector<double>> &get_sampled_xyzi() const;

    };
}