#pragma once
#ifdef USE_MCTS

#include <cmath>
#include <memory>
#include <stack>
#include <list>
#include <sys/stat.h>

#include <ros/ros.h>
#include <torch/script.h>

#include "planner_map_interfaces/ros_utils.h"
#include "ipp_planners/Planner.h"
#include "ipp_planners/InfoMapTrack.h"
#include "ipp_planners/TreeNode.h"
#include "ipp_belief/belief_manager.h"
#include "ipp_belief/information.h"
#include "ipp_belief/trackers.h"

inline bool check_file_exists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

namespace ipp
{

    class MCTSNode : public TreeNode
    {
    protected:
        // a pointer for polymorphism. a unique pointer to denote ownership
        // TODO: for now for ICRA, we are only doing InfoMapTrack, not InfoMapSearch
        ros::NodeHandle nh;
        std::unique_ptr<InfoMapTrack> info_map;

    public:
        double edge_value = 0;                  // value of the edge from the parent to this node
        unsigned int num_edge_observations = 0; // how many observations were used to update the edge value
        double future_value = 0;                // estimated value of the future
        double information = 0;                 // informatino is accumulated edge values
        double n_visits = 0;
        double node_var_prior = 0; // computed during rollout
        MCTSNode *parent = nullptr;
        std::vector<std::unique_ptr<MCTSNode>> children;

        double get_value()
        {
            return edge_value + future_value / n_visits;
        }

        /**
         * @brief Construct a new MCTSNode object. WARNING: info_map not set yet, must use set_info_map()
         *
         * @param si
         */
        MCTSNode(ros::NodeHandle &nh, const ob::SpaceInformationPtr &si)
            : nh(nh),
              TreeNode(si),
              info_map(std::make_unique<InfoMapTrack>(nh)) {}

        virtual ~MCTSNode() {}

        MCTSNode *empty_clone() override
        {
            if (!this->info_map)
            {
                throw std::runtime_error("info_map is null");
            }
            auto node = new MCTSNode(this->nh, this->si_);
            return node;
        }

        void add_child(std::unique_ptr<MCTSNode> child)
        {
            children.push_back(std::move(child));
        }

        void copy_info_map_params_from(MCTSNode &other)
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
        InfoMapTrack *get_info_map()
        {
            if (info_map)
                return info_map.get();
            else
                ROS_WARN("Asked to get_info_map but no info map to return");
        }

        void set_info_map(std::unique_ptr<InfoMapTrack> info_map)
        {
            this->info_map = std::move(info_map);
        }

        void set_belief(tracking::ParticleFiltersBelief &belief)
        {
            if (info_map)
                info_map->set_particle_belief_manager(belief);
            else
                throw std::runtime_error("Asked to set_belief but no info map to set belief");
        }
    };
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

    /**
     * @brief Records node's future value for training data for Learned MCTS variants
     *
     */
    class MCTSDataRecorder
    {
        std::string tracker_filename_;
        std::string state_filename_;
        std::string path_name_;
        bool activated_;

    public:
        MCTSDataRecorder(std::string name_append = "unknown")
        {
            tracker_filename_ = "tracker_list_" + name_append;
            state_filename_ = "state_" + name_append;
            path_name_ = "./train_data/";
            activated_ = false;
        }
        virtual ~MCTSDataRecorder() {}
        void record_activate()
        {
            activated_ = true;
        }
        void set_data_path(std::string path_name)
        {
            path_name_ = path_name;
        }
        void set_tracker_filename(std::string filename)
        {
            tracker_filename_ = filename;
        }
        void set_state_filename(std::string filename)
        {
            state_filename_ = filename;
        }
        void record_particles(MCTSNode *current_node)
        {
            if (!activated_)
            {
                return;
            }
            auto trackers = current_node->get_info_map()->particle_belief_manager.id_to_trackers;
            // ROS_INFO_STREAM("node tracker amount is " << trackers.size());
            std::ofstream tracker_out;
            tracker_out.open(path_name_ + tracker_filename_ + "_n_visits=" + std::to_string((int)current_node->n_visits) + ".csv");
            // remove the label line if needed
            tracker_out << "id"
                        << ", "
                        << "x"
                        << ", "
                        << "y"
                        << ", "
                        << "heading"
                        << ", "
                        << "speed"
                        << ", "
                        << "ang_v"
                        << "\n";
            for (const auto &[id, tracker] : trackers)
            {
                auto &particle_list = tracker.get_particles();
                // tracker_out << id << "\n";
                // ROS_INFO_STREAM("particle size is " << particle_list.size());
                for (size_t p_index = 0; p_index < particle_list.size(); p_index++)
                {
                    tracker_out << particle_list[p_index].get_id() << ", " << particle_list[p_index].get_x() << ", "
                                << particle_list[p_index].get_y() << ", " << particle_list[p_index].get_heading() << ", "
                                << particle_list[p_index].get_speed() << ", " << particle_list[p_index].get_angular_velocity() << "\n";
                }
            }
            tracker_out.close();
        }

        void record_status(MCTSNode *current_node)
        {
            if (!activated_)
            {
                return;
            }
            std::ofstream status_out;
            status_out.open(path_name_ + state_filename_ + ".csv");
            auto *s_ = current_node->state->as<XYZPsiStateSpace::StateType>();
            status_out << "x"
                       << ", "
                       << "y"
                       << ", "
                       << "z"
                       << ", "
                       << "heading"
                       << ", "
                       << "budget"
                       << ", "
                       << "init_var"
                       << "\n";
            status_out << s_->getX() << ", " << s_->getY() << ", " << s_->getZ() << ", " << s_->getPsi() << ", " << current_node->budget_remaining << ", ";
        }
        void record_variance(double variance)
        {
            if (!activated_)
            {
                return;
            }
            std::ofstream status_out;
            status_out.open(path_name_ + state_filename_ + ".csv", std::fstream::app);
            status_out << variance << "\n";
        }
        void record_value(double mcts_gain)
        {
            if (!activated_)
            {
                return;
            }
            std::ofstream status_out;
            status_out.open(path_name_ + state_filename_ + ".csv", std::fstream::app);
            status_out << mcts_gain << "\n";
        }
    };

    // MCTS abstract base class
    class MCTS : public Planner
    {
    protected:
        std::random_device rd;
        std::mt19937 gen;
        std::uniform_real_distribution<> distribution_psi;
        // Parameters
        int max_rollout_depth;    // read from rosparam
        unsigned int num_actions; // read from rosparam
        bool gen_train_data;      // read from rosparam
        const double exploration_weight = std::sqrt(2);

        // State
        MCTSNode tree_root;
        MCTSNode *leaf_node_of_best_path;

        unsigned int num_nodes_expanded = 0;
        unsigned int current_tree_depth = 0;
        std::vector<MCTSNode *> expanded_nodes;

        // Visual
        bool should_vis_rollouts = false;

        void enforce_track_only(ros::NodeHandle &nh)
        {
            bool is_track_task = ros_utils::get_param<bool>(nh, "track");
            bool is_search_task = ros_utils::get_param<bool>(nh, "search");
            if (is_search_task || !is_track_task)
            {
                ROS_ERROR_STREAM("MCTS is not implemented for search task, only track task. Please launch with search:=false track:=true");
                throw std::runtime_error("MCTS is not implemented for search task");
            }
        }

    public:
        MCTS(ros::NodeHandle &nh)
            : Planner(nh),
              tree_root(nh, XYZPsi_Space),
              distribution_psi(std::uniform_real_distribution<>(0, 2 * PI)),
              gen(std::mt19937(rd())),
              max_rollout_depth(ros_utils::get_param<int>(nh, "max_rollout_depth")),
              num_actions(ros_utils::get_param<int>(nh, "num_actions")),
              gen_train_data(ros_utils::get_param<bool>(nh, "gen_train_data")),
              exploration_weight(ros_utils::get_param<double>(nh, "exploration_weight")),
              should_vis_rollouts(ros_utils::get_param<bool>(nh, "vis_rollouts"))
        {
            enforce_track_only(nh);
        }

        virtual ~MCTS() {}

        /* =============================
         * ---- METHODS TO OVERRIDE ----
         * ============================= */

        /**
         * @brief Estimate the potential future value of a node. MUST be overriden.
         */
        virtual double rollout(MCTSNode &node) = 0;

        /**
         * @brief Choose an action to take from the given node. The new node should contain the new drone state
         * (without the world transition, the world transition is computed for you). The new node should point to the from_node as its parent.
         * MUST be overriden.
         *
         * This action node doesn't have to satisfy constraints. The return of this function is fed into the shared make_feasible_action_node
         * method that constrains the action for you.
         *
         * @param from_node node to take an action from
         * @param action_idx the action index out of the total number of actions. action index ranges between [0, this->num_actions - 1]
         * @return std::unique_ptr<MCTSNode>
         */
        virtual std::unique_ptr<MCTSNode> make_action_node(MCTSNode &from_node, int action_idx) = 0;

        /* ===================================
         * ---- SHARED BASE CLASS METHODS ----
         * =================================== */
        virtual void save_plan_request_params(const planner_map_interfaces::PlanRequest &msg)
        {
            Planner::save_plan_request_params(msg); // uncomment me when copy pasting to subclass
        }

        /**
         * @brief
         *
         * @param info_map the current info map
         * @param start_pose
         * @param budget
         * @param force_from_scratch
         * @return true
         * @return false
         */
        bool replan_setup(InfoMap &info_map_, std::vector<double> start_pose, double budget, bool force_from_scratch)
        {
            auto info_map = dynamic_cast<InfoMapTrack &>(info_map_);
            // Add start node to tree based on the given start pose
            ob::ScopedState<XYZPsiStateSpace> start_n(XYZPsi_Space);
            start_n->setXYZ(Eigen::Vector3d(start_pose[0], start_pose[1], start_pose[2]));
            start_n->setPsi(start_pose[3]);

            XYZPsi_Space->getStateSpace()->copyState(this->tree_root.state, start_n.get());

            // setup our root
            this->tree_root.name = "R";
            this->tree_root.budget_remaining = budget;
            this->tree_root.set_info_map(info_map.clone());
            this->tree_root.information = 0;
            // this->tree_root.children.clear(); // delete everybodyyy. TODO: make MCTS be able to somehow keep existing children if force_from_scratch = false
            // clear old stuff
            for (auto &child : this->tree_root.children)
            {
                child.reset();
            }
            this->tree_root.children.clear();
            unsigned int num_nodes_expanded = 0;
            unsigned int current_tree_depth = 0;
            this->expanded_nodes.clear();
            this->expand(this->tree_root);

            this->leaf_node_of_best_path = &tree_root; // Set starting node as best path. we no longer know what the best path is from here, so reset to where we are

            double var_prior = tracking::calculate_belief_variance(tree_root.get_info_map()->particle_belief_manager);
            this->tree_root.node_var_prior = var_prior;
            ROS_INFO_STREAM("Initial total variance is " << var_prior << "->" << tracking::transform_variance(var_prior));

            // print out initial variances
            auto infomaptrack = dynamic_cast<InfoMapTrack *>(&info_map);
            if (infomaptrack != nullptr)
            {
                for (auto [id, tracker] : infomaptrack->particle_belief_manager.get_id_to_trackers())
                {
                    double target_variance = tracking::calculate_trace_variance(tracker);
                    ROS_INFO_STREAM("Target id " << id << " variance: " << target_variance << "-->" << tracking::transform_variance(target_variance));
                }
            }

            // ROS_INFO_STREAM("Initial information gain of staying still set to " << leaf_node_of_best_path->information << " and has cost " << leaf_node_of_best_path->cost.value());
            ROS_INFO_STREAM("Exploration weight is " << exploration_weight);
            return true;
        }

        bool replan_loop(InfoMap &info_map_, std::vector<double> start_pose, double budget, bool force_from_scratch) override
        {
            ROS_DEBUG_STREAM("Replan loop");
            auto info_map = dynamic_cast<InfoMapTrack &>(info_map_);
            MCTSNode *current_node = &tree_root;

            // get to the best leaf node
            bool is_a_leaf_node = false;
            while (!(is_a_leaf_node = current_node->children.empty()))
            {
                current_node = this->select_child_highest_ucb1(current_node);
            }
            ROS_DEBUG_STREAM("Found leaf node " << current_node->name << " with " << current_node->n_visits << " visits");

            // evaluate it
            // is the n_i value for current 0?
            if (current_node->n_visits > 0)
            {
                ROS_DEBUG_STREAM("Expanding node with " << this->num_actions << " actions");
                this->expand(*current_node);
                ROS_INFO_STREAM("In total expanded " << this->num_nodes_expanded << " nodes, current depth is " << this->current_tree_depth);
                // assign current node to first child to roll it out
                current_node = current_node->children.at(0).get();
            }
            // ROLLOUT
            ROS_DEBUG_STREAM("Rolling out " << current_node->name << " with cost " << current_node->cost.value());
            this->rollout_and_backpropagate(*current_node);

            // // save best path
            // option 1: best path is the one that has the highest VALUE
            this->leaf_node_of_best_path = this->get_leaf_node_of_best_path();
            // option 2: best path is the one with the best accumulated INFORMATION. not sure which is better
            // if (leaf_node_of_best_path->name == "R" || current_node->information > leaf_node_of_best_path->information)
            // {
            //     leaf_node_of_best_path = current_node;
            //     ROS_INFO_STREAM("NEW BEST PATH! Reward is " << this->leaf_node_of_best_path->information << " and has cost " << leaf_node_of_best_path->cost.value());
            // }
            return true;
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
            double var_post = tracking::calculate_belief_variance(this->leaf_node_of_best_path->get_info_map()->particle_belief_manager);
            ROS_INFO_STREAM("Expected variance at end of planned path is " << var_post << "->" << tracking::transform_variance(var_post));

            if (this->gen_train_data)
            {
                std::string curr_stamp = std::to_string(ros::Time::now().toNSec());
                ROS_WARN_STREAM("recording tree info ");
                std::vector<MCTSNode *> node_list = this->get_expanded_nodes();
                for (size_t i = 0; i < node_list.size(); i++)
                {
                    MCTSNode *curr_node = node_list[i];
                    if (curr_node->name == "R")
                    {
                        ROS_DEBUG_STREAM("Skip recording root");
                        continue;
                    }
                    ROS_DEBUG_STREAM("recording node No." << i);
                    MCTSDataRecorder recorder("teardown_node_" + std::to_string(i) + "_time_" + curr_stamp);
                    recorder.record_activate();
                    recorder.record_status(curr_node);
                    recorder.record_particles(curr_node);
                    recorder.record_variance(curr_node->node_var_prior);
                    double remaining_time = this->max_plan_budget / this->desired_speed;
                    double future_value_avg_over_remaining_time = curr_node->future_value / remaining_time;
                    recorder.record_value(future_value_avg_over_remaining_time);
                }
            }
            return true;
        }

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
                // ROS_DEBUG_STREAM("Node " << child->name << " ucb1_value: " << score);
                if (score > best_score)
                {
                    best_score = score;
                    best_child = child.get();
                }
            }
            // ROS_DEBUG_STREAM("Selected child name=" << best_child->name << " with ucb1_value " << best_score);
            return best_child;
        }

        /**
         * @brief For each available action from the given, add a new state to the tree
         *
         *
         * @param node
         */
        virtual void expand(MCTSNode &node)
        {
            this->expanded_nodes.push_back(&node);
            ROS_INFO_STREAM("Expanding node " << node.name << " with value E" << node.edge_value << "+F" << node.future_value << "=" << node.get_value());
            // current = first new child
            // rollout current
            for (int i = 0; i < this->num_actions; i++)
            {
                try
                {
                    auto new_node = this->act_and_transition_to_new_node(node, i);
                    if (new_node->depth > this->current_tree_depth) // note this statement has to come before the std::move below
                    {
                        this->current_tree_depth = new_node->depth;
                        ROS_INFO_STREAM("New tree depth is " << this->current_tree_depth);
                    }
                    node.add_child(std::move(new_node));
                }
                catch (NodeStuckException e)
                {
                    ROS_WARN_STREAM("Node stuck, skipping in expand");
                }
            }
            this->num_nodes_expanded += 1;
        }

        /**
         * @brief Calls make_feasible_action_node() to get a new drone state and uses it to perform the state transition. Returns the resulting node containing the new belief
         *
         * @param from_node
         * @return std::pair<std::unique_ptr<MCTSNode>, double>; the new node and the gain from the transition
         */
        std::unique_ptr<MCTSNode> act_and_transition_to_new_node(MCTSNode &from_node, int action_idx)
        {
            /* ---- SAMPLE ACTION---- */

            // Sample an action from the belief
            // If we reach a terminal state, return the value of the state
            // Otherwise, continue rollout
            // ROS_DEBUG_STREAM("Sampling action");
            std::unique_ptr<MCTSNode> new_node = make_feasible_action_node(from_node, action_idx);
            int attempts = 1, max_attempts = 6;
            while (!new_node && attempts < max_attempts)
            {
                ROS_DEBUG_STREAM("Chosen action/node was infeasible, trying again.");
                new_node = make_feasible_action_node(from_node, action_idx);
                attempts++;
                if (attempts == max_attempts)
                {
                    char warn_msg[] = "Chosen action/node was infeasible many times, giving up.";
                    ROS_WARN_STREAM(warn_msg);
                    throw NodeStuckException(warn_msg);
                }
            }

            /* ---- STATE TRANSITION ---- */

            // ROS_DEBUG_STREAM("Computing state transition");
            // TODO: but not until after ICRA, make this cleaner so that infomaptrack handles all the information stuff. this depends on a unified Belief class between Search and Track

            auto [observations, time_deltas] = discretize_observations(*new_node, this->desired_speed, this->sensor_params, 1, from_node.get_info_map()->observation_discretization_distance);

            double transition_value;
            tracking::ParticleFiltersBelief transition_belief;
            if (ros_utils::get_param<std::string>(this->nh, "information_metric") == "particles")
            {
                auto [transition_value_, transition_belief_] = tracking::calculate_particle_info_gain(
                    from_node.get_info_map()->get_particle_belief_manager(),
                    observations,
                    time_deltas);
                transition_value = transition_value_;
                transition_belief = transition_belief_;
            }
            else if (ros_utils::get_param<std::string>(this->nh, "information_metric") == "variance_delta")
            {
                auto [transition_value_, transition_belief_] = tracking::calculate_variance_delta(
                    from_node.get_info_map()->get_particle_belief_manager(),
                    observations,
                    time_deltas,
                    tracking::transform_variance_info_gain);
                transition_value = transition_value_;
                transition_belief = transition_belief_;
            }
            else if (ros_utils::get_param<std::string>(this->nh, "information_metric") == "variance_accumulated")
            {
                auto [average_variance, transition_belief_] = tracking::calculate_variance_accumulated(
                    from_node.get_info_map()->get_particle_belief_manager(),
                    observations,
                    time_deltas);
                transition_value = average_variance * -1; // reward a.k.a. value = -variance
                transition_belief = transition_belief_;
            }
            else
            {
                ROS_ERROR_STREAM("Unknown information metric " << ros_utils::get_param<std::string>(this->nh, "information_metric"));
                throw std::runtime_error("Invalid information metric");
            }

            new_node->set_belief(transition_belief);
            new_node->edge_value = transition_value;
            new_node->num_edge_observations = observations.size();
            new_node->information = from_node.information + transition_value;
            new_node->depth = from_node.depth + 1;
            new_node->parent = &from_node;
            new_node->name = from_node.name + "." + std::to_string(action_idx);
            if (transition_belief.get_num_active_trackers() == 0)
            {
                new_node->is_in_closed_set = true;
            }
            // ROS_DEBUG_STREAM("State transition set.");
            return std::move(new_node);
        }

        /**
         * @brief Constrains a raw action node created by make_action_node() into a feasible action node that satisfies extent constraints
         *
         * @param from_node
         * @return std::unique_ptr<MCTSNode>
         */
        std::unique_ptr<MCTSNode> make_feasible_action_node(MCTSNode &from_node, int action_idx)
        {
            // Sample an action from the belief
            // If we reach a terminal state, return the value of the state
            // Otherwise, continue rollout
            std::unique_ptr<MCTSNode> raw_action_node = make_action_node(from_node, action_idx);
            // this function says it returns a TreeNode type, but we know it's a MCTSNode because it uses the .clone() function
            TreeNode *feasible_tree_node = extend_from_start_node_toward_goal_node(*(from_node.get_info_map()), &from_node, raw_action_node.get());
            MCTSNode *feasible_mcts_node = dynamic_cast<MCTSNode *>(feasible_tree_node);
            feasible_mcts_node->copy_info_map_params_from(from_node);

            if (feasible_mcts_node == nullptr)
            {
                // ROS_WARN_STREAM("Could not cast feasible tree node to MCTSNode");
                return std::unique_ptr<MCTSNode>(nullptr);
            }
            return std::unique_ptr<MCTSNode>(feasible_mcts_node);
        }

        /**
         * @brief Rollout to estimate the future value of the given node, assign the value, set the visit count to 1, and backpropagate the value
         *
         * @param node
         */
        void rollout_and_backpropagate(MCTSNode &node)
        {
            if (node.n_visits != 0)
            {
                throw std::invalid_argument("Rollout called on a node that has already been visited");
            }
            node.future_value = rollout(node);
            node.n_visits = 1;
            backpropagate(&node);
        }

        /**
         * @brief Backpropagate total value and n visits up the tree.
         *
         * @param node node to backpropagate from. Must have total value and n_visits already set.
         */
        void backpropagate(MCTSNode *node)
        {
            // ROS_DEBUG_STREAM("Backpropagating from node of depth " << node->depth);
            while (node != nullptr && node->parent != nullptr)
            {
                auto parent = node->parent;
                parent->future_value += node->get_value();
                parent->n_visits += 1;
                node = parent;
            }
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

    // random rollouts to termination, to estimate the future value of actions sampled from the information map
    class RolloutInformedActionMCTS : public MCTS
    {
        unsigned int rollout_vis_id = 0;
    public:
        ros::Publisher rollout_publisher;

        void visualize_rollout(MCTSNode &node)
        {
            auto &edge = node.edge_trochoid;
            visualization_msgs::Marker m;
            m.header.frame_id = "local_enu";
            m.header.stamp = ros::Time();
            m.ns = "rollout";
            // m.frame_locked = true;
            m.id = this->rollout_vis_id++;
            if (this->rollout_vis_id >= std::numeric_limits<int>::max())
                this->rollout_vis_id = 0;
            m.lifetime = ros::Duration(0.2);
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            m.scale.x = 30.0;
            m.color.a = 0.5;
            m.color.r = 1.0;
            m.color.g = 0.4;
            auto &path_states = node.edge_trochoid.getStates();
            for (auto it = path_states.begin(); it < path_states.end() - 2; it += 5)
            {
                auto s1 = *it;
                auto s2 = *(it + 1);

                geometry_msgs::Point child_point;
                auto node_pt = s1->as<XYZPsiStateSpace::StateType>();
                child_point.x = node_pt->getX();
                child_point.y = node_pt->getY();
                child_point.z = node_pt->getZ();

                // std_msgs::ColorRGBA new_color;
                // new_color.a = 1.0;
                // new_color.r = new_color.g = 1.0;
                // m.colors.push_back(new_color);
                m.points.push_back(child_point);
            }
            rollout_publisher.publish(m);
        }

        RolloutInformedActionMCTS(ros::NodeHandle &nh)
            : MCTS(nh), rollout_publisher(nh.advertise<visualization_msgs::Marker>("rollout", 10))
        {
        }

        virtual ~RolloutInformedActionMCTS() {}
        /* =============================
         * ----- OVERRIDEN METHODS -----
         * ============================= */
        /**
         * @brief Rollout a sequence of actions until termination to estimate the value of the node
         *
         * @param node
         * @return double
         */
        double rollout(MCTSNode &from_node) override
        {
            // Sample an action from the belief
            // If we reach a terminal state, return the value of the state
            // Otherwise, continue rollout
            MCTSNode *current_node = &from_node;
            int num_rollouts = 0;
            auto *s_ = current_node->state->as<XYZPsiStateSpace::StateType>();
            ROS_DEBUG_STREAM("Rolling out node depth= " << current_node->depth
                                                        << " with budget remaining " << current_node->budget_remaining
                                                        << " from " << s_->getX() << " " << s_->getY() << " " << s_->getZ());
            double var_prior = tracking::calculate_belief_variance(from_node.get_info_map()->particle_belief_manager);

            double rollout_value = 0;
            unsigned int total_num_observations = 0; // only for variance_average calculation
            std::uniform_int_distribution<int> action_idx_dist(0, this->num_actions - 1);
            do
            {
                s_ = current_node->state->as<XYZPsiStateSpace::StateType>();
                // ROS_DEBUG_STREAM("Rolling out node depth= " << current_node->depth << " from " << s_->getX() << " " << s_->getY() << " " << s_->getZ());
                // move() frees the previous state, we don't need it when rolling out
                try
                {
                    auto new_node = act_and_transition_to_new_node(*current_node, action_idx_dist(this->gen));
                    if (this->should_vis_while_planning && this->should_vis_rollouts)
                    {
                        this->visualize_rollout(*new_node);
                    }
                    rollout_value += new_node->edge_value;

                    if (num_rollouts > 0) // make sure not to delete the from_node
                        delete current_node;
                    current_node = new_node.release();
                }
                catch (NodeStuckException e)
                {
                    ROS_WARN_STREAM("Node stuck at " << s_->getX() << " " << s_->getY() << " " << s_->getZ());
                    break;
                }
                num_rollouts++;
            } while (!current_node->is_in_closed_set && num_rollouts < this->max_rollout_depth);

            if (num_rollouts > this->max_rollout_depth && !current_node->is_in_closed_set)
            {
                ROS_INFO_STREAM("Careful: Rollout reached max depth but node is not in closed set (Budget remaining =  "
                                << this->max_plan_budget - current_node->cost.value() << "). This implies some myopicness.");
            }

            ROS_DEBUG_STREAM(std::fixed << "Rollout finished with " << num_rollouts << " steps. "
                                        << " Rollout value is " << rollout_value);
            return rollout_value;
        }

        /**
         * @brief Use the InfoMap to directly sample a location from the map, and turn it into a node.
         *
         * @return TreeNode nullptr if invalid sample,
         */
        std::unique_ptr<MCTSNode> make_action_node(MCTSNode &from_node, int action_idx) override
        {
            auto from_state = from_node.state->as<XYZPsiStateSpace::StateType>();
            double x, y;

            auto particle_belief_manager = from_node.get_info_map()->particle_belief_manager;
            auto valid_ship_ids = particle_belief_manager.get_tracker_ids();
            // only do explicit mean for targets between a certain variance
            valid_ship_ids = from_node.get_info_map()->filter_target_ids_within_variance(valid_ship_ids, 5, 10000);
            std::sort(valid_ship_ids.begin(), valid_ship_ids.end());
            if (action_idx < valid_ship_ids.size())
            {
                // return the mean of the tracker
                auto tracker = particle_belief_manager.get_tracker(valid_ship_ids[action_idx]);
                auto mean = tracker.get_mean_particle();

                auto xy = from_node.get_info_map()->solve_soonest_intersection_drone_to_particle(
                    from_state->getX(), from_state->getY(), this->desired_speed, mean);
                x = xy.first, y = xy.second;
            }
            else
            {
                auto xy = from_node.get_info_map()->sample_xy(from_node);
                x = xy.first, y = xy.second;
            }

            if (isnan(x) || isnan(y))
            {
                ROS_ERROR_STREAM("Got invalid sample from InfoMap");
                throw std::runtime_error("Invalid sample");
            }

            ob::ScopedState<XYZPsiStateSpace> new_state(this->XYZPsi_Space);
            // Sample the altitude and angle
            double altitude = this->flight_height;
            double psi = this->distribution_psi(gen);

            auto optimal_xyzpsi = this->find_viewing_point(
                std::vector<double>{
                    from_state->getX(),
                    from_state->getY(),
                    from_state->getZ(),
                    from_state->getPsi()},
                std::make_pair(x, y));

            // Set the values in the new node
            new_state->setX(optimal_xyzpsi[0]);
            new_state->setY(optimal_xyzpsi[1]);
            new_state->setZ(optimal_xyzpsi[2]);
            new_state->setPsi(optimal_xyzpsi[3]);
            // this->xyzi_samples.push_back(std::vector<double>({new_state->getX(), new_state->getY(), new_state->getZ(), 1})); // xyz and information

            // ROS_DEBUG_STREAM("Sampled node at X::" << x << " Y::" << y << " Z::" << altitude << " Psi::" << psi);

            // we have to use a unique_ptr instead of by value because MCTSNode contains a unique_ptr to InfoMap, whose copy constructor has been deleted (can only move unique_ptrs)
            auto sampled_node = std::make_unique<MCTSNode>(this->nh, this->XYZPsi_Space);
            sampled_node->get_info_map()->desired_speed = this->desired_speed;

            // copy the new state into the node
            // ROS_DEBUG_STREAM("Copy state " << x << " Y::" << y << " Z::" << altitude << " Psi::" << psi);
            this->XYZPsi_Space->getStateSpace()->copyState(sampled_node->state, new_state.get());

            return sampled_node;
        }
    };

    // use a neural network to shortcut estimating the future value of actions sampled from the information map
    class LearnedInformedActionMCTS : public RolloutInformedActionMCTS
    {
        torch::jit::script::Module module; // the torchscript module

    public:
        int max_num_particles;
        LearnedInformedActionMCTS(ros::NodeHandle &nh)
            : RolloutInformedActionMCTS(nh),
              max_num_particles(ros_utils::get_param<int>(nh, "/ipp_belief/num_particles"))
        {
            std::string module_path = ros_utils::get_param<std::string>(nh, "limcts_torchscript_file");
            if (!check_file_exists(module_path))
            {
                ROS_ERROR_STREAM("Could not find torchscript file at " << module_path);
                throw std::runtime_error("Could not find torchscript file");
            }
            ROS_INFO_STREAM("Loading torchscript module from " << module_path);
            try
            {
                // Deserialize the ScriptModule from a file using torch::jit::load().
                this->module = torch::jit::load(module_path);
            }
            catch (const c10::Error &e)
            {
                ROS_ERROR_STREAM("Failed to load torchscript file: " << e.what());
                throw;
            }
        }

        /**
         * @brief Uses a trained neural network to estimate the future value of the given node
         *
         * @param from_node
         * @return double
         */
        double rollout(MCTSNode &from_node) override
        {

            auto *s_ = from_node.state->as<XYZPsiStateSpace::StateType>();
            ROS_DEBUG_STREAM("Performing future-value inference on node of depth= " << from_node.depth << " from " << s_->getX() << " " << s_->getY() << " " << s_->getZ());

            // Step 1: accumulate the particles and drone input into vectors

            std::vector<float> drone_vec;
            std::vector<float> particle_vec;

            double current_variance = tracking::calculate_belief_variance(from_node.get_info_map()->particle_belief_manager);
            // x, y, heading, budget, variance
            drone_vec.push_back((float)s_->getX());
            drone_vec.push_back((float)s_->getY());
            drone_vec.push_back((float)s_->getPsi());
            drone_vec.push_back((float)from_node.budget_remaining);
            drone_vec.push_back((float)current_variance);

            float
                min_x = std::numeric_limits<double>::max(),
                min_y = std::numeric_limits<double>::max(),
                max_x = std::numeric_limits<double>::min(),
                max_y = std::numeric_limits<double>::min();

            // put all the particles in our input
            for (auto [id, tracker] : from_node.get_info_map()->particle_belief_manager.id_to_trackers)
            {
                auto &particles = tracker.get_particles();
                for (auto &particle : particles)
                {
                    if (particle.get_x() < min_x)
                        min_x = particle.get_x();
                    if (particle.get_x() > max_x)
                        max_x = particle.get_x();
                    if (particle.get_y() < min_y)
                        min_y = particle.get_y();
                    if (particle.get_y() > max_y)
                        max_y = particle.get_y();

                    particle_vec.push_back((float)particle.get_x());
                    particle_vec.push_back((float)particle.get_y());
                    particle_vec.push_back((float)particle.get_heading());
                    particle_vec.push_back((float)particle.get_speed());
                }
                int num_remaining = this->max_num_particles - particles.size();
                for (int i = 0; i < num_remaining; i++)
                {
                    particle_vec.push_back(0.0f);
                    particle_vec.push_back(0.0f);
                    particle_vec.push_back(0.0f);
                    particle_vec.push_back(0.0f);
                }
            }

            // we decided in our experiments to limit the number of trackers to 10
            int num_trackers_remaining = 10 - from_node.get_info_map()->particle_belief_manager.id_to_trackers.size();
            for (int i = 0; i < num_trackers_remaining; i++)
            {
                for (int j = 0; j < 4 * this->max_num_particles; j++)
                {
                    particle_vec.push_back(0.0f);
                }
            }

            // Step 2: center everthing about (0,0), as we've done during training

            // 2a: center the drone about (0,0)
            drone_vec[0] -= (max_x + min_x) / 2;
            drone_vec[1] -= (max_y + min_y) / 2;

            // 2b: center the particles about (0,0)
            for (int i = 0; i < particle_vec.size(); i += 4)
            {
                particle_vec[i] -= (max_x + min_x) / 2.0f;
                particle_vec[i + 1] -= (max_y + min_y) / 2.0f;
            }

            // Step 3: convert the vector into a tensor and IValue inputs

            // https://github.com/pytorch/pytorch/issues/18337#issuecomment-475709658
            // https://www.simonwenkel.com/notes/software_libraries/pytorch/data_transfer_to_and_from_pytorch.html#stdvector-totorchtensor-and-back
            torch::Tensor drone_tensor = torch::from_blob(drone_vec.data(), {1, (int)drone_vec.size()}, torch::kFloat32);
            torch::Tensor particle_tensor = torch::from_blob(particle_vec.data(), {1, (int)particle_vec.size()}, torch::kFloat32);

            std::vector<torch::jit::IValue> inputs;
            inputs.push_back(drone_tensor);
            inputs.push_back(particle_tensor);

            // Step 4: feed the data through our neural network module
            at::Tensor output = module.forward(inputs).toTensor();
            double estimated_future_value = output.item<double>();

            // Step 5: unnormalize by the remaining time in this node
            double time = from_node.budget_remaining / this->desired_speed;
            double estimated_future_accumulated_value = estimated_future_value * time;

            return estimated_future_accumulated_value;
        }
    };

    /**
     * @brief A baseline MCTS that uses a predetermined set of primitive actions.
     *
     */
    class RolloutPrimitiveActionMCTS : public RolloutInformedActionMCTS
    {
        double action_radius = 600;

        // num_actions size of <x, y, psi>
        std::vector<std::tuple<double, double, double>> motion_primitivies;

    public:
        RolloutPrimitiveActionMCTS(ros::NodeHandle &nh)
            : RolloutInformedActionMCTS(nh)
        {
            this->num_actions = 14;
        }

        virtual ~RolloutPrimitiveActionMCTS() {}

        /**
         * @brief Constrain a radian angle between -pi and pi
         *
         * @param x
         * @return double
         */
        double constrain_angle(double x)
        {
            x = fmod(x + M_PI, 2 * M_PI);
            if (x < 0)
                x += 2 * M_PI;
            return x - M_PI;
        }

        std::unique_ptr<MCTSNode> make_action_node(MCTSNode &from_node, int action_idx) override
        {

            auto *from_state = from_node.state->as<XYZPsiStateSpace::StateType>();
            double from_x = from_state->getX();
            double from_y = from_state->getY();
            double from_psi = from_state->getPsi();

            double min_angle = -2 * M_PI / 3; // -135 degrees
            double max_angle = 2 * M_PI / 3;  // 135 degrees

            int num_angles = this->num_actions / 2;
            int angle_idx = action_idx % num_angles;
            // ROS_DEBUG_STREAM("action_idx: " << action_idx);
            double radius = (action_idx > angle_idx) ? this->action_radius * 3.0 : this->action_radius;
            // ROS_DEBUG_STREAM("angle_idx: " << angle_idx);
            double angle = min_angle + (max_angle - min_angle) * (double) angle_idx / (num_angles - 1.) + from_psi;
            // ROS_DEBUG_STREAM("angle unnormalized: " << angle - from_psi);

            auto motion_primitive = std::make_tuple(radius * cos(angle), radius * sin(angle), angle);

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
    };

    class LearnedPrimitiveActionMCTS : public RolloutPrimitiveActionMCTS
    {
        torch::jit::script::Module module; // the torchscript module

    public:
        int max_num_particles;
        LearnedPrimitiveActionMCTS(ros::NodeHandle &nh)
            : RolloutPrimitiveActionMCTS(nh),
              max_num_particles(ros_utils::get_param<int>(nh, "/ipp_belief/num_particles"))
        {
            std::string module_path = ros_utils::get_param<std::string>(nh, "lpmcts_torchscript_file");
            if (!check_file_exists(module_path))
            {
                ROS_ERROR_STREAM("Could not find torchscript file at " << module_path);
                throw std::runtime_error("Could not find torchscript file");
            }
            ROS_INFO_STREAM("Loading torchscript module from " << module_path);
            try
            {
                // Deserialize the ScriptModule from a file using torch::jit::load().
                this->module = torch::jit::load(module_path);
            }
            catch (const c10::Error &e)
            {
                ROS_ERROR_STREAM("Failed to load torchscript file: " << e.what());
                throw;
            }
        }

        /**
         * @brief Uses a trained neural network to estimate the future value of the given node
         *
         * @param from_node
         * @return double
         */
        double rollout(MCTSNode &from_node) override
        {

            auto *s_ = from_node.state->as<XYZPsiStateSpace::StateType>();
            ROS_DEBUG_STREAM("Performing future-value inference on node of depth= " << from_node.depth << " from " << s_->getX() << " " << s_->getY() << " " << s_->getZ());

            // Step 1: accumulate the particles and drone input into vectors

            std::vector<float> drone_vec;
            std::vector<float> particle_vec;

            double current_variance = tracking::calculate_belief_variance(from_node.get_info_map()->particle_belief_manager);
            // x, y, heading, budget, variance
            drone_vec.push_back((float)s_->getX());
            drone_vec.push_back((float)s_->getY());
            drone_vec.push_back((float)s_->getPsi());
            drone_vec.push_back((float)from_node.budget_remaining);
            drone_vec.push_back((float)current_variance);

            float
                min_x = std::numeric_limits<double>::max(),
                min_y = std::numeric_limits<double>::max(),
                max_x = std::numeric_limits<double>::min(),
                max_y = std::numeric_limits<double>::min();

            // put all the particles in our input
            for (auto [id, tracker] : from_node.get_info_map()->particle_belief_manager.id_to_trackers)
            {
                auto &particles = tracker.get_particles();
                for (auto &particle : particles)
                {
                    if (particle.get_x() < min_x)
                        min_x = particle.get_x();
                    if (particle.get_x() > max_x)
                        max_x = particle.get_x();
                    if (particle.get_y() < min_y)
                        min_y = particle.get_y();
                    if (particle.get_y() > max_y)
                        max_y = particle.get_y();

                    particle_vec.push_back((float)particle.get_x());
                    particle_vec.push_back((float)particle.get_y());
                    particle_vec.push_back((float)particle.get_heading());
                    particle_vec.push_back((float)particle.get_speed());
                }
                int num_remaining = this->max_num_particles - particles.size();
                for (int i = 0; i < num_remaining; i++)
                {
                    particle_vec.push_back(0.0f);
                    particle_vec.push_back(0.0f);
                    particle_vec.push_back(0.0f);
                    particle_vec.push_back(0.0f);
                }
            }

            // we decided in our experiments to limit the number of trackers to 10
            int num_trackers_remaining = 10 - from_node.get_info_map()->particle_belief_manager.id_to_trackers.size();
            for (int i = 0; i < num_trackers_remaining; i++)
            {
                for (int j = 0; j < 4 * this->max_num_particles; j++)
                {
                    particle_vec.push_back(0.0f);
                }
            }

            // Step 2: center everthing about (0,0), as we've done during training

            // 2a: center the drone about (0,0)
            drone_vec[0] -= (max_x + min_x) / 2;
            drone_vec[1] -= (max_y + min_y) / 2;

            // 2b: center the particles about (0,0)
            for (int i = 0; i < particle_vec.size(); i += 4)
            {
                particle_vec[i] -= (max_x + min_x) / 2.0f;
                particle_vec[i + 1] -= (max_y + min_y) / 2.0f;
            }

            // Step 3: convert the vector into a tensor and IValue inputs

            // https://github.com/pytorch/pytorch/issues/18337#issuecomment-475709658
            // https://www.simonwenkel.com/notes/software_libraries/pytorch/data_transfer_to_and_from_pytorch.html#stdvector-totorchtensor-and-back
            torch::Tensor drone_tensor = torch::from_blob(drone_vec.data(), {1, (int)drone_vec.size()}, torch::kFloat32);
            torch::Tensor particle_tensor = torch::from_blob(particle_vec.data(), {1, (int)particle_vec.size()}, torch::kFloat32);

            std::vector<torch::jit::IValue> inputs;
            inputs.push_back(drone_tensor);
            inputs.push_back(particle_tensor);

            // Step 4: feed the data through our neural network module
            at::Tensor output = module.forward(inputs).toTensor();
            double estimated_future_value = output.item<double>();

            // Step 5: unnormalize by the remaining time in this node
            double time = from_node.budget_remaining / this->desired_speed;
            double estimated_future_accumulated_value = estimated_future_value * time;

            return estimated_future_accumulated_value;
        }
    };

}
#endif