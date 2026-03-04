#pragma once
#include <cmath>
#include <memory>
#include <stack>
#include <list>
#include <sys/stat.h>

#include <ros/ros.h>
// #include <torch/torch.h>
// #include <torch/script.h>

#include "planner_map_interfaces/camera_projection.h"
#include "ipp_planners/Planner.h"
#include "ipp_planners/MCTSNode.h"

inline bool check_file_exists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}


namespace ipp
{
    // TODO: Need to create a common MCTS base class and derive MCTS track and MCTS search 
    // Right now the following exception class and other functionality is duplicated
    class NodeStuckException : public std::exception
    {
        private:
            char *message;

        public:
            NodeStuckException(char *msg) : message(msg) {}
            char *what()
            {
                return message;
            }
    };

    class MCTSSearch : public Planner
    {
        protected:
            std::random_device rd;
            std::mt19937 gen;
            // Parameters
            double mcts_extend_radius;  // read from rosparam
            double mcts_extend_angle;  // read from rosparam
            double mcts_extend_dist;  // read from rosparam
            int mcts_branches_each_side;  // read from rosparam
            int max_rollout_depth;    // read from rosparam
            unsigned int num_actions; // read from rosparam
            const double exploration_weight = std::sqrt(2);
            std::string rollout_type = "primitive";

            // State
            MCTSNode tree_root;
            MCTSNode *leaf_node_of_best_path;

            unsigned int num_nodes_expanded = 0;
            unsigned int current_tree_depth = 0;
            std::vector<MCTSNode *> expanded_nodes;

            // torch::jit::script::Module rollout_model; 
            // torch::jit::script::Module map_encoder_model;

        public:
            std::vector<double> pose_to_plan_from;
            std::vector<std::vector<double>> search_bounds;
            std::vector<std::vector<double>> planned_path;
            std::vector<std::vector<double>> sampled_points;
            std::vector<std::vector<double>> view_points;
            std::vector<og::PathGeometric> planned_path_segments;

            double total_path_length;

            std::string planner_name = "mcts_search";

            MCTSSearch(ros::NodeHandle &nh, ros::NodeHandle &pnh) 
                :   Planner(nh, pnh),
                    tree_root(nh, pnh, XYZPsi_Space),
                    gen(std::mt19937(rd())),
                    max_rollout_depth(ros_utils::get_param<int>(pnh, "max_rollout_depth")),
                    num_actions(ros_utils::get_param<int>(pnh, "num_actions")),
                    exploration_weight(ros_utils::get_param<double>(pnh, "exploration_weight")),
                    mcts_extend_radius(ros_utils::get_param<double>(pnh, "mcts_extend_radius")),
                    mcts_extend_angle(ros_utils::get_param<double>(pnh, "mcts_extend_angle")),
                    mcts_extend_dist(ros_utils::get_param<double>(pnh, "mcts_extend_dist")),
                    rollout_type(ros_utils::get_param<std::string>(pnh, "rollout_type")),
                    mcts_branches_each_side(ros_utils::get_param<double>(pnh, "mcts_branches_each_side")) {}

            ~MCTSSearch() {}

            double ucb1_score(MCTSNode &node)
            {
                if (node.n_visits == 0)
                {
                    return std::numeric_limits<double>::max();
                }
                return node.get_value() / (this->max_plan_budget / this->desired_speed) + this->exploration_weight * std::sqrt(std::log(this->tree_root.n_visits) / node.n_visits);
            }
            
            MCTSNode *select_child_highest_ucb1(MCTSNode *node)
            {
                MCTSNode *best_child = nullptr;
                double best_score = std::numeric_limits<double>::lowest();
                if (node->children.empty())
                {
                    ROS_WARN("Trying to select best child of node with no children");
                }
                for (auto &child : node->children)
                {
                    double score = ucb1_score(*child);
                    if (score > best_score)
                    {
                        best_score = score;
                        best_child = child.get();
                    }
                }
                return best_child;
            }

            bool replan_teardown(InfoMap &info_map, std::vector<double> start_pose, double budget, bool force_from_scratch)
            {
                ROS_INFO_STREAM("Total nodes expanded: " << this->num_nodes_expanded << ". Deepest tree depth is: " << this->current_tree_depth);
                double time = this->max_plan_budget / this->desired_speed;
                ROS_INFO_STREAM(
                    "The best path total reward = " << this->leaf_node_of_best_path->information
                                                    << " (normed over total time " << time << "s = " << this->leaf_node_of_best_path->information / time << ")"
                                                    << ", with expected future value is " << this->leaf_node_of_best_path->future_value
                                                    << " (normed over total time " << time << "s = " << this->leaf_node_of_best_path->future_value / time << ")"
                                                    << " estimated with " << this->leaf_node_of_best_path->n_visits << " visits."
                                                    << " Overall expected reward is " << this->leaf_node_of_best_path->information + this->leaf_node_of_best_path->future_value
                                                    << " normed over total time as " << (this->leaf_node_of_best_path->information + this->leaf_node_of_best_path->future_value) / time);
                ROS_INFO_STREAM("The best path total budget used is: " << this->leaf_node_of_best_path->cost.value());

                return true;
            }

            bool replan_setup(InfoMap &info_map_, std::vector<double> start_pose, double budget, bool force_from_scratch)
            {
                auto info_map = dynamic_cast<InfoMapSearch &>(info_map_);

                // Add start node to tree based on the given start pose
                ob::ScopedState<XYZPsiStateSpace> start_n(XYZPsi_Space);
                start_n->setXYZ(Eigen::Vector3d(start_pose[0], start_pose[1], start_pose[2]));
                start_n->setPsi(start_pose[3]);
                XYZPsi_Space->getStateSpace()->copyState(this->tree_root.state, start_n.get());
                // setup our root
                this->tree_root.name = "R";
                this->tree_root.budget_remaining = budget;
                this->tree_root.set_info_map(std::make_unique<InfoMapSearch>(info_map));
                this->tree_root.information = 0;

                for (auto &child : this->tree_root.children)
                {
                    child.reset();
                }
                this->tree_root.children.clear();
                unsigned int num_nodes_expanded = 0;
                unsigned int current_tree_depth = 0;
                this->expanded_nodes.clear();
                this->expand(this->tree_root);

                if (rollout_type == "learned")
                {
                    // // Load the learned model
                    // std::string rollout_model_path = ros_utils::get_param<std::string>(pnh, "rollout_model_path");
                    // std::string map_encoder_model_path = ros_utils::get_param<std::string>(pnh, "map_encoder_model_path");
                    // if (!check_file_exists(rollout_model_path) ) {
                    //     ROS_ERROR_STREAM("Could not find torchscript file at " << rollout_model_path);
                    //     throw std::runtime_error("Could not find torchscript file");
                    // }

                    // if (!check_file_exists(map_encoder_model_path) ) {
                    //     ROS_ERROR_STREAM("Could not find torchscript file at " << map_encoder_model_path);
                    //     throw std::runtime_error("Could not find torchscript file");
                    // }

                    // ROS_INFO_STREAM("Loading torchscript models...");
                    
                    // try {
                    //     // Deserialize the ScriptModule from a file using torch::jit::load().
                    //     this->rollout_model = torch::jit::load(rollout_model_path);
                    //     this->map_encoder_model = torch::jit::load(map_encoder_model_path);
                    // } catch (const c10::Error &e) {
                    //     ROS_ERROR_STREAM("Failed to load torchscript file: " << e.what());
                    //     throw;
                    // }

                }


                // Set starting node as best path. we no longer know what the best path is from here,
                // so reset to where we are
                this->leaf_node_of_best_path = &tree_root;

                ROS_INFO_STREAM("[" << planner_name << "] EXTEND RADIUS: " << mcts_extend_radius); 
                ROS_INFO_STREAM("[" << planner_name << "] EXTEND ANGLE: " << mcts_extend_angle);
                ROS_INFO_STREAM("[" << planner_name << "] MAX ROLLOUT DEPTH: " << max_rollout_depth);
                ROS_INFO_STREAM("[" << planner_name << "] NUM ACTIONS: " << num_actions);
                ROS_INFO_STREAM("[" << planner_name << "] EXPLORATION WEIGHT: " << exploration_weight);
                ROS_INFO_STREAM("[" << planner_name << "] BRANCHES EACH SIDE: " << mcts_branches_each_side);

                ROS_INFO_STREAM("Initial information gain of staying still set to " << leaf_node_of_best_path->information 
                                << " and has cost " << leaf_node_of_best_path->cost.value());

                ROS_INFO_STREAM("[" << planner_name << "] Setup ");
                
                return true;   
            }

            bool replan_loop(InfoMap &info_map_, std::vector<double> start_pose, double budget, bool force_from_scratch)
            {
                // ROS_INFO_STREAM("Beginning replan from ::" << start_pose[0] << "::" << start_pose[1] << "::" << start_pose[2]);
                auto info_map = dynamic_cast<InfoMapSearch &>(info_map_);
                MCTSNode *current_node = &tree_root;

                // Selection Phase: Find the leaf node with highest UCB score

               bool is_a_leaf_node = false;
               while (!(is_a_leaf_node = current_node->children.empty())) {
                // ROS_INFO_STREAM("NOT A LEAF NODE");
                current_node = this->select_child_highest_ucb1(current_node);
               }

                // ROS_INFO_STREAM("Found leaf node " << current_node->name << " with " << current_node->n_visits << " visits");

                // Expansion phase
                if (current_node->n_visits > 0)
                {
                    // ROS_DEBUG_STREAM("Expanding node with " << this->num_actions << " actions");
                    this->expand(*current_node);
                    // ROS_INFO_STREAM("In total expanded " << this->num_nodes_expanded << " nodes, current depth is " << this->current_tree_depth);
                    // assign current node to first child to roll it out
                    current_node = current_node->children.at(0).get();
                }

                // ROLLOUT
                // ROS_INFO_STREAM("Rolling out " << current_node->name << " with cost " << current_node->cost.value());
                this->rollout_and_backpropagate(*current_node);

                if (leaf_node_of_best_path->name == "R" || current_node->information > leaf_node_of_best_path->information)
                {
                    leaf_node_of_best_path = current_node;
                    ROS_INFO_STREAM("NEW BEST PATH! Reward is " << this->leaf_node_of_best_path->information << " and has cost " << leaf_node_of_best_path->cost.value());
                }
            return true;
                
            }

            void expand(MCTSNode &node)
            {
                this->expanded_nodes.push_back(&node);
                // ROS_INFO_STREAM("Expanding node " << node.name << " with value E" << node.edge_value << "+F" << node.future_value << "=" << node.get_value());
                
                for (int i = 0; i < this->num_actions; i++)
                {
                    try
                    {
                        auto new_node = this->act_and_transition_to_new_node(node, i);
                        if (new_node->depth > this->current_tree_depth) // note this statement has to come before the std::move below
                        {
                            this->current_tree_depth = new_node->depth;
                            // ROS_INFO_STREAM("New tree depth is " << this->current_tree_depth);
                        }
                        node.add_child(std::move(new_node));
                    }
                    catch (NodeStuckException e)
                    {
                        // ROS_WARN_STREAM("Node stuck, skipping in expand");
                    }
                }
                this->num_nodes_expanded += 1;
            }

            std::unique_ptr<MCTSNode> act_and_transition_to_new_node(MCTSNode &from_node, int action_idx)
            {
                /* 1. Request a sampled state from the belief. 
                    2. If sampled state is feasible, request the feasible node
                    3. Compute new node meta data and return new node  */
                std::unique_ptr<MCTSNode> new_node = make_feasible_action_node(from_node, action_idx);

                int attempts = 1, max_attempts = 6;
                while (!new_node && attempts < max_attempts)
                {
                    // ROS_INFO_STREAM("Chosen action/node was infeasible, trying again.");
                    new_node = make_feasible_action_node(from_node, action_idx);
                    attempts++;
                    if (attempts == max_attempts)
                    {
                        char warn_msg[] = "Chosen action/node was infeasible many times, giving up.";
                        ROS_WARN_STREAM(warn_msg);
                        throw NodeStuckException(warn_msg);
                    }
                }

                // Compute new node meta data
                new_node->parent = &from_node;
                new_node->name = from_node.name + "." + std::to_string(action_idx);
                new_node->depth = from_node.depth + 1;
                auto parent_info_map = from_node.get_info_map();
                std::unique_ptr<InfoMapSearch> uniqueInfoMapSearch = std::make_unique<InfoMapSearch>(*parent_info_map);
                new_node->set_info_map(std::move(uniqueInfoMapSearch));

                double transition_value = compute_transition_value(*new_node, from_node);
                new_node->edge_value = transition_value;
                new_node->information = from_node.information + transition_value;

                return new_node;
            }

            double compute_transition_value(MCTSNode &new_node, MCTSNode &parent_node)
            {
                return new_node.get_info_map()->calc_from_node_to_new_node_value(new_node, parent_node, include_edge);
            }

            std::unique_ptr<MCTSNode> make_feasible_action_node(MCTSNode &from_node, int action_idx)
            {
                // Sample an action from the belief
                // If we reach a terminal state, return the value of the state
                // Otherwise, continue rollout
                std::unique_ptr<MCTSNode> raw_action_node = nullptr;
                if (rollout_type == "informed")
                {
                    raw_action_node = make_informed_action_node(from_node, action_idx);
                }
                else if (rollout_type == "primitive")
                {
                    raw_action_node = make_action_node(from_node, action_idx);
                }
                // std::unique_ptr<MCTSNode> raw_action_node = make_action_node(from_node, action_idx);

                // this function says it returns a TreeNode type, but we know it's a MCTSNode because it uses the .clone() function
                TreeNode *feasible_tree_node = extend_from_start_node_toward_goal_node(*(from_node.get_info_map()), &from_node, raw_action_node.get());
                
                MCTSNode *feasible_mcts_node = dynamic_cast<MCTSNode *>(feasible_tree_node);
                if (feasible_mcts_node == nullptr) {
                    return std::unique_ptr<MCTSNode>(nullptr);
                }

                auto from_info_map = from_node.get_info_map();
                std::unique_ptr<InfoMapSearch> uniqueInfoMapSearch = std::make_unique<InfoMapSearch>(*from_info_map);
                feasible_mcts_node->set_info_map(std::move(uniqueInfoMapSearch));

                feasible_mcts_node->info_map->bounds = from_node.info_map->bounds;

                return std::unique_ptr<MCTSNode>(feasible_mcts_node);
            }

            /**
             * @brief Implements an informed sampler that uses the information map to sample a new state
             * 
             * @param MCTSNode &from_node: The node to start the rollout from
             * @param int action_idx: The index of the action to take
             * @return std::unique_ptr<MCTSNode> new_node: The new node
             */
            std::unique_ptr<MCTSNode> make_informed_action_node(MCTSNode &from_node, int action_idx)
            {
                auto from_info_map = from_node.get_info_map();
                auto xy = from_info_map->sample_xy(this->tree_root);
                double x = xy.first, y = xy.second;

                if (isnan(x) || isnan(y))
                {
                    throw std::runtime_error("Invalid sample");
                }

                auto distribution_psi = std::uniform_real_distribution<double>(-M_PI, M_PI - 0.0001);
                double psi = distribution_psi(this->gen);

                auto new_node = std::unique_ptr<MCTSNode>(from_node.empty_clone());
                this->XYZPsi_Space->getStateSpace()->copyState(new_node->state, from_node.state);
                auto *new_state = new_node->state->as<XYZPsiStateSpace::StateType>();
                new_state->setX(x);
                new_state->setY(y);
                new_state->setPsi(psi);
                this->XYZPsi_Space->getStateSpace()->enforceBounds(new_node->state);
                return new_node;
            }


            /**
             * @brief Implements an action sampler that uses motion primitives to sample a new state
             * 
             * @param MCTSNode &from_node: The node to start the rollout from
             * @param int action_idx: The index of the action to take
             * @return std::unique_ptr<MCTSNode> new_node: The new node
             */
            std::unique_ptr<MCTSNode> make_action_node(MCTSNode &from_node, int action_idx)
            {
                // Uses motion primitives to sample new state
                auto *from_state = from_node.state->as<XYZPsiStateSpace::StateType>();
                double from_x = from_state->getX();
                double from_y = from_state->getY();
                double from_psi = from_state->getPsi();

                double min_angle = -2 * M_PI / 3; // -135 degrees
                double max_angle = 2 * M_PI / 3;  // 135 degrees

                int num_angles = this->num_actions / 2;
                int angle_idx = action_idx % num_angles;

                double radius = (action_idx > angle_idx) ? this->mcts_extend_radius * 3.0 : this->mcts_extend_radius;

                double angle = min_angle + (max_angle - min_angle) * (double) angle_idx / (num_angles - 1.) + from_psi;

                auto motion_primitive = std::make_tuple(radius * cos(angle), radius * sin(angle), angle);

                // With MCTSNode empty_clone changes this should now be safe
                auto new_node = std::unique_ptr<MCTSNode>(from_node.empty_clone());
                this->XYZPsi_Space->getStateSpace()->copyState(new_node->state, from_state);
                auto *new_state = new_node->state->as<XYZPsiStateSpace::StateType>();

                double new_x = from_x + std::get<0>(motion_primitive);
                double new_y = from_y + std::get<1>(motion_primitive);
                double new_psi = constrain_angle(std::get<2>(motion_primitive));
                new_state->setX(new_x);
                new_state->setY(new_y);
                // Z is the same for now, already copied
                new_state->setPsi(new_psi);
                return new_node;
            }

            double constrain_angle(double x)
            {
                x = fmod(x + M_PI, 2 * M_PI);
                if (x < 0)
                    x += 2 * M_PI;
                return x - M_PI;
            }

            void rollout_and_backpropagate(MCTSNode &node)
            {
                if (node.n_visits != 0)
                {
                    throw std::invalid_argument("Rollout called on a node that has already been visited");
                }
                node.future_value = rollout(node);
                node.n_visits ++;
                backpropagate(&node);
            }

            void backpropagate(MCTSNode *node)
            {
                while (node != nullptr && node->parent != nullptr)
                {
                    auto parent = node->parent;
                    parent->future_value += node->get_value();
                    parent->n_visits += 1;
                    node = parent;
                }
            }

            /**
             * @brief Perform a rollout from the given node
             * 
             * @param MCTSNode &from_node: The node to start the rollout from
             * @return double rollout_value: The value of the rollout
             */
            double rollout(MCTSNode &from_node) 
            {
                MCTSNode *current_node = &from_node;
                int num_rollouts = 0;
                auto *s_ = current_node->state->as<XYZPsiStateSpace::StateType>();
                double rollout_value = 0;


                if (rollout_type == "learned") {
                    // std::vector<torch::jit::IValue> inputs;
                    // auto map = current_node->get_info_map();                    

                }

                else {
                    std::uniform_int_distribution<int> action_idx_dist(0, this->num_actions - 1);
                    std::vector<MCTSNode*> rollout_nodes;

                    do
                    {
                        s_ = current_node->state->as<XYZPsiStateSpace::StateType>();
                        try
                        {
                            auto new_node = act_and_transition_to_new_node(*current_node, action_idx_dist(this->gen));

                            rollout_value += new_node->information;
                            rollout_nodes.push_back(new_node.release());
                            current_node = rollout_nodes.back();
                        }
                        catch (NodeStuckException e)
                        {
                            break;
                        }
                        num_rollouts++;
                    } while (num_rollouts < this->max_rollout_depth && current_node);

                    // deallocate the pointers to the nodes, but not the nodes themselves
                    for (MCTSNode* node : rollout_nodes)
                    {
                        delete node;
                    }
                }

                return rollout_value;
            }

            MCTSNode *get_leaf_node_of_best_path()
            {
                MCTSNode *current_node = &tree_root;
                while (current_node->children.size() > 0)
                {
                    double best_child_value = -std::numeric_limits<double>::infinity();
                    // get the node with the highest value
                    for (auto &child : current_node->children)
                    {
                        if (child->get_value() > best_child_value)
                        {
                            current_node = child.get();
                            best_child_value = child->get_value();
                        }
                    }
                }
                return current_node;
            }

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

            /**
             * @brief Performs a DFS to get all the nodes in the MCTS tree
             *
             * @return const std::vector<const MCTSNode *>&
             */
            const std::vector<MCTSNode *> get_all_nodes(int min_num_visits = 0)
            {
                std::vector<MCTSNode *> all_nodes;
                MCTSNode *node = &tree_root;
                // depth first search
                std::stack<MCTSNode *> stack;
                stack.push(node);
                while (!stack.empty())
                {
                    node = stack.top();
                    stack.pop();
                    all_nodes.push_back(node);
                    for (auto &child : node->children)
                    {
                        if (child->n_visits >= min_num_visits)
                        {
                            stack.push(child.get());
                        }
                    }
                }
                return all_nodes;
            }

            const std::vector<MCTSNode *> get_expanded_nodes() const
            {
                return expanded_nodes;
            }
        };

}

// #endif
