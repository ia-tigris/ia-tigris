#pragma once

#include <cmath>
#include <memory>
#include <stack>
#include <list>
#include <sys/stat.h>

#include <ros/ros.h>

#include "ipp_planners/TreeNode.h"
#include "ipp_planners/InfoMapSearch.h"

namespace ipp
{

    class MCTSNode : public TreeNode
    {
    protected:
        // a pointer for polymorphism. a unique pointer to denote ownership
        // TODO: consider specializing this node for search map variants if needed.
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

    public:
        double edge_value = 0;                  // Info gain from parent to current node
        double future_value = 0;                // estimated value of the future
        double information = 0;                 // Accumulated info gain from tree root to current node
        double n_visits = 0;
        double node_var_prior = 0; // computed during rollout
        MCTSNode *parent = nullptr;
        std::vector<std::unique_ptr<MCTSNode>> children;
        std::unique_ptr<InfoMapSearch> info_map;

        double get_value()
        {
            return edge_value + future_value / n_visits;
        }

        /**
         * @brief Construct a new MCTSNode object. WARNING: info_map not set yet, must use set_info_map()
         *
         * @param si
         */
        MCTSNode(ros::NodeHandle &nh, ros::NodeHandle &pnh, const ob::SpaceInformationPtr &si)
            : nh(nh),
              pnh(pnh),
              TreeNode(si),
              info_map(std::make_unique<InfoMapSearch>(nh, pnh)) {}

        virtual ~MCTSNode() {}

        MCTSNode *empty_clone() override
        {
            if (!this->info_map)
            {
                throw std::runtime_error("info_map is null");
            }
            
            auto new_node = std::make_unique<MCTSNode>(this->nh, this->pnh, this->si_);
            return new_node.release();
        }

        void add_child(std::unique_ptr<MCTSNode> child)
        {
            children.push_back(std::move(child));
        }

        // get info map. uses getters to warn in case nullptr
        InfoMapSearch *get_info_map()
        {
            if (info_map)
                return info_map.get();
            else
                ROS_WARN("Asked to get_info_map but no info map to return");
            return nullptr;
        }

        void set_info_map(std::unique_ptr<InfoMapSearch> info_map)
        {
            this->info_map = std::move(info_map);
        }
    };
}
// #endif
