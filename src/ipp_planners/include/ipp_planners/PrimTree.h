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

namespace ipp
{
    class PrimTree : public Planner
    {
    protected:
        TreeNode *tree_root;
        TreeNode *leaf_node_of_best_path;
        std::queue<TreeNode*> nodes_to_expand;
        std::vector<TreeNode*> all_nodes;
        double total_budget;
        double desired_speed;
        ompl::NearestNeighborsGNAT<TreeNode *> nn_data_structure;
    public:
        double primtree_extend_radius;
        double primtree_extend_angle;
        double primtree_extend_dist;

        int primtree_branches_each_side;
        std::string planner_name = "PrimTree";

        PrimTree(ros::NodeHandle &nh, ros::NodeHandle &pnh): 
            Planner(nh, pnh),
            primtree_extend_radius(ros_utils::get_param<double>(pnh, "primtree_extend_radius")),
            primtree_extend_angle(ros_utils::get_param<double>(pnh, "primtree_extend_angle")),
            primtree_extend_dist(ros_utils::get_param<double>(pnh, "primtree_extend_dist")),
            primtree_branches_each_side(ros_utils::get_param<double>(pnh, "primtree_branches_each_side"))
        {
            planner_name = ros_utils::get_param<std::string>(pnh, "planner");

            this->nn_data_structure.setDistanceFunction([this](TreeNode *a, TreeNode *b)
                                                    {  this->XYZPsi_Space->enforceBounds(a->state);
                                                        this->XYZPsi_Space->enforceBounds(b->state);
                                                        if (std::abs(a->state->as<XYZPsiStateSpace::StateType>()->getPsi() - -PI) < 1e-5){
                                                            a->state->as<XYZPsiStateSpace::StateType>()->setPsi(-PI + 1e-5);
                                                        }
                                                        if (std::abs(b->state->as<XYZPsiStateSpace::StateType>()->getPsi() - -PI) < 1e-5){
                                                            b->state->as<XYZPsiStateSpace::StateType>()->setPsi(-PI + 1e-5);
                                                        }
                                                        return this->XYZPsi_Space->distance(a->state, b->state); });
        }

        ~PrimTree()
        {
            std::vector<TreeNode *> all_nodes;
            nn_data_structure.list(all_nodes);
            for (auto it : all_nodes)
            {
                delete it;
            }
            this->nn_data_structure.clear();
        }

        bool replan_setup(InfoMap &info_map, std::vector<double> start_pose, double budget, bool force_from_scratch)
        {
            // Initialize the tree
            auto tree_start_time = ompl::time::now();
            this->update_tree(start_pose, info_map, force_from_scratch);
            ROS_WARN_STREAM("Time to build tree: " << duration_cast<duration<double>>(Clock::now() - tree_start_time).count());
            return true;
        }

        bool replan_loop(InfoMap &info_map, std::vector<double> start_pose, double budget, bool should_clear_tree)
        {
            // ROS_WARN_STREAM("expand list length " << nodes_to_expand.size());
            if (nodes_to_expand.size() == 0)
            {
                throw std::invalid_argument("expand list empty");
            }
            TreeNode* node_to_add = nodes_to_expand.front();
            TreeNode* node_add_from = node_to_add->parent;
            expand_children_from_node(info_map, node_to_add);
            // This extend is done already when expanding
            // auto *add_node = this->extend_from_start_node_toward_goal_node(info_map, node_near, node_feasible, this->primtree_extend_dist);
            this->add_node_to_tree(info_map, node_to_add, node_add_from);
            nodes_to_expand.pop();
            // ROS_INFO_STREAM("adding node to tree");
            return true;
        }

        bool replan_teardown(InfoMap &info_map, std::vector<double> start_pose, double budget, bool force_from_scratch)
        {
            ROS_INFO_STREAM("Tree size " << this->nn_data_structure.size());
            ROS_INFO_STREAM("The best path total reward is: " << this->leaf_node_of_best_path->information);
            ROS_INFO_STREAM("The best path total budget used is: " << this->leaf_node_of_best_path->cost.value());
            while (!nodes_to_expand.empty())
            {
                delete nodes_to_expand.front();
                nodes_to_expand.pop();
            }
            
            return true;
        }

        bool add_node_to_tree(InfoMap &info_map, TreeNode *add_node, TreeNode *add_node_parent)
        {
            if (add_node != nullptr)
            {
                ROS_DEBUG_STREAM("Adding node to tree of depth " << add_node->depth);
                add_node->information = info_map.calc_child_to_root_value(*add_node, include_edge);
                if (isnan(add_node->information))
                {
                    ROS_WARN_STREAM("Information is nan");
                }
                ROS_DEBUG_STREAM("Added node with info gain " << add_node->information);

                auto *state_ptr = add_node->state->as<XYZPsiStateSpace::StateType>();
                // this->xyzi_samples.push_back(std::vector<double>({state_ptr->getX(), state_ptr->getY(), state_ptr->getZ(), add_node->information})); // xyz and information

                add_node_parent->add_child(add_node);
                this->nn_data_structure.add(add_node); 

                // check if it's the new best path
                if (add_node->information > this->leaf_node_of_best_path->information)
                {
                    this->leaf_node_of_best_path = add_node;
                    ROS_INFO_STREAM("New best path info gain is " << this->leaf_node_of_best_path->information << " and has cost " << leaf_node_of_best_path->cost.value());
                }
            }
            return true;
        }

        void update_tree(std::vector<double> &start_pose, InfoMap &info_map, bool should_clear_tree)
        {
            // Find node in tree that matches start pose
            TreeNode *node_ptr;
            // this can be nullptr if none of the nodes on the path were close to where the drone is
            if (should_clear_tree || !(node_ptr = this->find_closest_tree_node_to_pose(start_pose)))
            {
                ROS_INFO_STREAM(
                    "Starting a fresh plan because "
                    << (should_clear_tree ? "asked to clear tree" : "no node found in tree that was close to drone pose"));
                this->reset_tree_to_pose(start_pose, info_map);
            }
            else
            {
                this->recycle_tree_from_node(node_ptr, info_map);
                if (this->nn_data_structure.size() == 0)
                {
                    ROS_INFO_STREAM("Tree recycling cleared all nodes. Starting new plan");
                    this->reset_tree_to_pose(start_pose, info_map);
                }
            }
        }

        void recycle_tree_from_node(TreeNode *node_ptr, InfoMap &info_map)
        {
            ROS_INFO_STREAM("Tree size before recycle " << this->nn_data_structure.size());
            this->erase_nodes_before_start(node_ptr);
            this->nn_data_structure.clear();
            // TODO need to clear out the nodes that get deleted
            // Add the root node to tree
            // Add start node to tree
            node_ptr->parent = nullptr;
            node_ptr->budget_remaining = this->max_plan_budget;
            // double cost_reduction = node_ptr->cost.value();
            node_ptr->local_search_map_updates.clear();
            node_ptr->information = info_map.calc_child_to_root_value(*node_ptr, include_edge); // comment andrew: brady and I changed this
            // node_ptr->information = std::numeric_limits<double>::lowest();
            
            node_ptr->cost = ob::Cost(0);
            this->tree_root = node_ptr;
            expand_children_from_node(info_map, node_ptr);
            this->leaf_node_of_best_path = node_ptr; // Set starting node as best path
            // ROS_INFO_STREAM("Cost reduction is " << cost_reduction);
            // ROS_INFO_STREAM("New best path info gain is " << this->leaf_node_of_best_path->information << " and has cost " << leaf_node_of_best_path->cost.value());
            this->add_children_to_tree(info_map, node_ptr);
            ROS_INFO_STREAM("Size of recycled tree is " << this->nn_data_structure.size());
        }

        void add_children_to_tree(InfoMap &info_map, TreeNode *parent)
        {
            // Update the information and cost of the children and add to tree
            for (auto *child : parent->children)
            {
                // Update the information and cost of the child
                if (child->incremental_cost.value() == 0)
                {
                    ROS_WARN_STREAM("Child has incremental cost of 0");
                }

                child->cost = ob::Cost(parent->cost.value() + child->incremental_cost.value());
                child->budget_remaining = parent->budget_remaining - child->incremental_cost.value();

                if(child->cost.value() > max_plan_budget)
                {
                    ROS_DEBUG_STREAM("Child has cost " << child->cost.value() << " which is greater than the max plan budget " << max_plan_budget);
                    // remove child from parent children list
                    parent->children.erase(std::remove(parent->children.begin(), parent->children.end(), child), parent->children.end());
                    // delete child
                    delete_node_and_children(child);
                    return;
                }

                child->local_search_map_updates.clear(); // TODO technically could keep this and do a faster update
                child->information = info_map.calc_child_to_root_value(*child, include_edge);

                if (child->information > this->leaf_node_of_best_path->information)
                {
                    this->leaf_node_of_best_path = child;
                    // ROS_INFO_STREAM("New best path info gain is " << this->leaf_node_of_best_path->information << " and has cost " << leaf_node_of_best_path->cost.value());
                }

                // Add the child to the tree
                this->nn_data_structure.add(child);
                add_children_to_tree(info_map, child);
            }
        }

        void delete_node_and_children(TreeNode *parent)
        {
            for (auto *child : parent->children)
            {
                delete_node_and_children(child);
            }
            delete parent;
        }

        void erase_nodes_before_start(TreeNode *node_ptr)
        {
            if (node_ptr->parent == nullptr)
            {
                return;
            }

            // First remove node_ptr from parent's children so that it doesn't get deleted (yet)
            node_ptr->parent->children.erase(std::remove(node_ptr->parent->children.begin(), node_ptr->parent->children.end(), node_ptr), node_ptr->parent->children.end());

            // Collect nodes to be deleted
            std::vector<TreeNode *> nodes_to_delete;
            for (auto *child_node : node_ptr->parent->children)
            {
                if (child_node != node_ptr) // Double check
                {
                    nodes_to_delete.push_back(child_node);
                }
                else
                {
                    ROS_INFO("Skipped self node");
                }
            }

            // Delete nodes
            for (auto *node : nodes_to_delete)
            {
                erase_node_and_children(node);
            }

            erase_nodes_before_start(node_ptr->parent);
            delete node_ptr->parent;
            node_ptr->parent = nullptr;
        }

        void erase_node_and_children(TreeNode *node_ptr)
        {
            for (auto *child : node_ptr->children)
            {
                erase_node_and_children(child);
            }
            delete node_ptr;
        }

        void reset_tree_to_pose(std::vector<double> &start_pose, InfoMap &info_map)
        {
            // clear previous tree
            // loop over nn_data_structure and delete each node
            std::vector<TreeNode *> all_nodes;
            nn_data_structure.list(all_nodes);
            for (auto it : all_nodes)
            {
                delete it;
            }
            this->nn_data_structure.clear(); // releases memory. so I hope

            // Add start node to tree
            ob::ScopedState<XYZPsiStateSpace> start_n(XYZPsi_Space);
            Eigen::Vector3d start_v(start_pose[0], start_pose[1], start_pose[2]);
            start_n->setXYZ(start_v);
            start_n->setPsi(start_pose[3]);
            this->XYZPsi_Space->enforceBounds(start_n.get());

            auto *node = new TreeNode(XYZPsi_Space);
            XYZPsi_Space->getStateSpace()->copyState(node->state, start_n.get());
            this->tree_root = node;
            this->tree_root->budget_remaining = this->max_plan_budget;
            node->information = info_map.calc_child_to_root_value(*node, include_edge); // Change from neg infinity
            node->cost = ob::Cost(0);
            node->incremental_cost = ob::Cost(0);
            // node->information = std::numeric_limits<double>::lowest();
            this->leaf_node_of_best_path = node; // Set starting node as best path. we no longer know what the best path is from here, so reset to where we are
            ROS_INFO_STREAM("Initial information gain set to " << leaf_node_of_best_path->information << " and has cost " << leaf_node_of_best_path->cost.value());
            ROS_DEBUG_STREAM("Add root node X::" << start_n.get()->getX() << " Y::" << start_n.get()->getY() << " Z::" << start_n.get()->getZ() << " Psi::" << start_n.get()->getPsi());

            // add its children to list
            expand_children_from_node(info_map, node);
            
            this->nn_data_structure.add(node);
        }

        TreeNode* find_closest_tree_node_to_pose(std::vector<double> &start_pose)
        {
            // TODO Could either remove the other nodes or copy the good part of the tree.
            // Not sure which one is faster
            TreeNode *node_ptr = this->leaf_node_of_best_path;
            while (node_ptr != nullptr)
            {
                auto *node_pt = node_ptr->state->as<XYZPsiStateSpace::StateType>();
                if (abs(node_pt->getX() - start_pose[0]) < 1e-3 &&
                    abs(node_pt->getY() - start_pose[1]) < 1e-3 &&
                    abs(node_pt->getZ() - start_pose[2]) < 1e-3 &&
                    abs(node_pt->getPsi() - start_pose[3]) < 1e-2)
                {
                    ROS_INFO_STREAM("Found start node in tree at X::" << node_pt->getX() << " Y::" << node_pt->getY() << " Z::" << node_pt->getZ() << " Psi::" << node_pt->getPsi());
                    ROS_INFO_STREAM("Desired start pose is at X::" << start_pose[0] << " Y::" << start_pose[1] << " Z::" << start_pose[2] << " Psi::" << start_pose[3]);
                    break;
                }
                node_ptr = node_ptr->parent;
            }
            return node_ptr;
        }

        void expand_children_from_node(InfoMap &info_map, TreeNode* parent_node)
        {
            // expand local motion primitives, like a fan shape pointing to the front, with number of branches as setting param
            // straight
            double turning_psi = 0.0;
            TreeNode* str_child = get_child_node(info_map, parent_node, turning_psi, primtree_extend_radius);
            nodes_to_expand.push(str_child);

            double step_angle = primtree_extend_angle / (2 * (double)primtree_branches_each_side);
            for (int i = 0; i < primtree_branches_each_side; i++)
            {
                // left
                double left_psi = step_angle * (double)(i + 1);
                TreeNode* left_child = get_child_node(info_map, parent_node, left_psi, primtree_extend_radius);
                nodes_to_expand.push(left_child);

                // right
                double right_psi = - step_angle * (double)(i + 1);
                TreeNode* right_child = get_child_node(info_map, parent_node, right_psi, primtree_extend_radius);
                nodes_to_expand.push(right_child);
            }
            return;
        }

        TreeNode* get_child_node(InfoMap &info_map, TreeNode* parent_node, double turning_psi, double radius)
        {
            auto* start_pos = parent_node->state->as<XYZPsiStateSpace::StateType>();
            double child_psi = start_pos->getPsi() + turning_psi;
            double child_x = start_pos->getX() + primtree_extend_radius * cos(child_psi);
            double child_y = start_pos->getY() + primtree_extend_radius * sin(child_psi);
            double child_z = start_pos->getZ();
            ob::ScopedState<XYZPsiStateSpace> child_n(XYZPsi_Space);
            Eigen::Vector3d child_v(child_x, child_y, child_z);
            child_n->setXYZ(child_v);
            child_n->setPsi(child_psi);
            auto *child_node = new TreeNode(XYZPsi_Space);
            XYZPsi_Space->getStateSpace()->copyState(child_node->state, child_n.get());
            
            TreeNode *feasible_child_node = this->extend_from_start_node_toward_goal_node(info_map, parent_node, child_node, this->primtree_extend_dist);

            delete child_node;

            return feasible_child_node;
        }

        const std::vector<TreeNode *> get_all_nodes() const
        {
            std::vector<TreeNode *> all_nodes;
            std::vector<TreeNode*> nonconst_nodes;
            nn_data_structure.list(nonconst_nodes);
            for (auto *node : nonconst_nodes)
            {
                all_nodes.push_back(node);
            }
            return all_nodes;
        }

        TreeNode* get_leaf_node_of_best_path() const
        {
            return leaf_node_of_best_path;
        };

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
    };
}
