#ifndef MOTION_H
#define MOTION_H

#include <memory>
#include <vector>
#include <limits>
#include <list>

#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/tools/config/SelfConfig.h>
#include <unordered_map>

#include "trochoids/trochoid_utils.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;

class TreeNode
{
public:
    // name the node
    std::string name = "";
    // pointer to state space
    ob::SpaceInformationPtr si_;
    /** \brief The parent node in the exploration tree */
    TreeNode *parent = nullptr;

    // XYZHeading of the start and end pose for the straight line segment of the trochoid path
    std::vector<double> straight_edge_start_pose;
    std::vector<double> straight_edge_end_pose;

    // The trochoid edge from the parent to this node
    og::PathGeometric edge_trochoid;

    /** \brief The state contained by the node */
    // pointer for polymorphism.
    // TODO: don't make this a pointer. instead make TreeNode a template
    ob::State *state;
    unsigned int depth = 0;
    double budget_remaining = 0; // remaining available budget from this node to the root
    /** \brief The cost up to this node */
    ob::Cost cost;
    /** \brief The incremental cost of this node's parent to this node (this is stored to save distance
     * computations in the updateChildCosts() method) */
    ob::Cost incremental_cost;

    // END STATE
    // Set to true if the budget has been reached at this node
    bool is_in_closed_set;

    // Information
    double information = 0; 
    double search_info = 0; // information from the search map specifically
    std::unordered_map<int, double> local_search_map_updates;

    /** \brief The set of motions descending from the current node */
    std::vector<TreeNode *> children;

    /** \brief Constructor that allocates memory for the state. This constructor automatically allocates
     * memory for \e state, \e cost, and \e incremental_cost */
    TreeNode(const ob::SpaceInformationPtr &si) : si_(si), state(si->allocState()), parent(nullptr), is_in_closed_set(false), information(0), edge_trochoid(si)
    {
    }

    virtual ~TreeNode(){
        this->si_->freeState(this->state);
    }

    // clone
    virtual TreeNode *empty_clone()
    {
        return new TreeNode(si_);
    }

    void add_child(TreeNode *child)
    {
        children.push_back(child);
    }

    std::list<TreeNode *> get_path_from_parent(int max_num_parents = std::numeric_limits<int>::max())
    {
        std::list<TreeNode *> path;
        TreeNode *current_node = this;
        int num_parents = 0;
        while (current_node->parent != nullptr && num_parents < max_num_parents)
        {
            path.push_front(current_node);
            current_node = current_node->parent;
            num_parents++;
        }
        return path;
    }

    std::list<TreeNode *> get_path_to_parent(TreeNode *parent){
        std::list<TreeNode *> path;
        TreeNode *current_node = this;
        path.push_front(current_node);
        while (current_node->parent != nullptr && current_node != parent){
            current_node = current_node->parent;
            path.push_front(current_node);
        }
        return path;
    }
};

#endif