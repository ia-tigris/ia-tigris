#include "ipp_planners/Tigris.h"

namespace ipp
{
    Tigris::Tigris(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : Planner(nh, pnh),
          extend_dist(ros_utils::get_param<double>(pnh, "extend_dist")),
          extend_radius(ros_utils::get_param<double>(pnh, "extend_radius")),
          prune_radius(ros_utils::get_param<double>(pnh, "prune_radius")),
          distribution_psi(std::uniform_real_distribution<>(-PI, PI - 0.0001)), // OMPL wants [-pi, pi), not [0, 2pi]
          gen(std::mt19937(rd()))
    {
        ROS_INFO("Initializing Tigris Planner");
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

        ROS_INFO("Finished initializing Tigris Planner");
    }

    Tigris::~Tigris()
    {
        std::vector<TreeNode *> all_nodes;
        nn_data_structure.list(all_nodes);
        for (auto it : all_nodes)
        {
            delete it;
        }
        this->nn_data_structure.clear(); 
    }

    bool Tigris::replan_setup(InfoMap &info_map, std::vector<double> start_pose, double budget, bool force_from_scratch)
    {
        // STEP 1: UPDATE THE TREE
        auto tree_start_time = ompl::time::now();
        this->update_tree(start_pose, info_map, force_from_scratch);
        ROS_INFO_STREAM("Time to build tree: " << duration_cast<duration<double>>(Clock::now() - tree_start_time).count());
        // STEP 2: DRAW SAMPLES FOR THE PLANNER UNTIL TIME RUNS OUT
        xyzi_samples.clear();
        num_samples_drawn = 0;
        return true;
    }

    /**
     * @brief The "main" function of the Tigris planner.
     *
     * @param info_map the current information map to plan against
     * @param start_pose size 4 vector, XYZpsi. start pose of the agent to plan from
     * @param budget
     * @param should_clear_tree whether the existing tree should be completely cleared
     */
    bool Tigris::replan_loop(
        InfoMap &info_map,
        std::vector<double> start_pose,
        double budget, bool should_clear_tree)
    {
        TreeNode* feasible_sampled_node = this->sample_node_within_budget(info_map, budget);

        if (feasible_sampled_node)
        {
            // extend all nearest nodes to the newly added node
            if (feasible_sampled_node == nullptr)
            {
                ROS_ERROR("Sampled node was null. Should have been caught?");
            }
            this->extend_neighbors_towards_node(info_map, feasible_sampled_node, budget);
            num_samples_drawn++;
        } else {
            ROS_DEBUG("Sampled node infeasible");
        }
        return true;
    }

    bool Tigris::replan_teardown(InfoMap &info_map, std::vector<double> start_pose, double budget, bool force_from_scratch)
    {
        ROS_INFO_STREAM("Number of samples drawn: " << this->num_samples_drawn);
        ROS_INFO_STREAM("Tree size " << this->nn_data_structure.size());
        ROS_INFO_STREAM("The best path total reward is: " << this->leaf_node_of_best_path->information);
        ROS_INFO_STREAM("The best path total budget used is: " << this->leaf_node_of_best_path->cost.value());

        // print all nodes
        // std::vector<TreeNode *> all_nodes;
        // nn_data_structure.list(all_nodes);
        // for (auto it : all_nodes)
        // {
        //     auto *state_ptr = it->state->as<XYZPsiStateSpace::StateType>();
        //     ROS_INFO_STREAM("Node at X::" << state_ptr->getX() << " Y::" << state_ptr->getY() << " Z::" << state_ptr->getZ() << " Psi::" << state_ptr->getPsi() << " has info " << it->information << " and cost " << it->cost.value());
        // }
        return true;
    }

    /**
     * @brief Decides how to update the tree. If manually chosen to clear the tree, or if no node in the tree
     * is close to the agent's pose, then we must completely reset the tree to where the drone is now.
     * Otherwise if a node on the tree is close to the agent's pose, we can recycle the tree from that node.
     */
    void Tigris::update_tree(std::vector<double> &start_pose, InfoMap &info_map, bool should_clear_tree)
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

    /**
     * @brief Clears the data structure and restarts the tree from this pose
     */
    void Tigris::reset_tree_to_pose(std::vector<double> &start_pose, InfoMap &info_map)
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
        this->nn_data_structure.add(node);
    }

    /**
     * @brief Finds the closest node along this->leaf_node_of_best_path's tree to the given point.
     *
     * @param start_pose XYZPsi coordinate (4-vector)
     * @return TreeNode* the closest node; warning: may be nullptr
     */
    TreeNode *Tigris::find_closest_tree_node_to_pose(std::vector<double> &start_pose)
    {
        // TODO Could either remove the other nodes or copy the good part of the tree.
        // Not sure which one is faster
        TreeNode *node_ptr = this->leaf_node_of_best_path;
        while (node_ptr != nullptr)
        {
            auto *node_pt = node_ptr->state->as<XYZPsiStateSpace::StateType>();
            double angle_diff = std::fabs(node_pt->getPsi() - trochoids::WrapToPi(start_pose[3]));
            angle_diff = (angle_diff > M_PI) ? (2 * M_PI - angle_diff) : angle_diff; //subtract 2PI to bring into range [-pi, pi]

            if (abs(node_pt->getX() - start_pose[0]) < 1e-3 &&
                abs(node_pt->getY() - start_pose[1]) < 1e-3 &&
                abs(node_pt->getZ() - start_pose[2]) < 1e-3 &&
                angle_diff < 1e-2)
            {
                ROS_INFO_STREAM("Found start node in tree at X::" << node_pt->getX() << " Y::" << node_pt->getY() << " Z::" << node_pt->getZ() << " Psi::" << node_pt->getPsi());
                ROS_INFO_STREAM("Desired start pose is at X::" << start_pose[0] << " Y::" << start_pose[1] << " Z::" << start_pose[2] << " Psi::" << start_pose[3]);
                break;
            }
            node_ptr = node_ptr->parent;
        }
        return node_ptr;
    }

    /**
     * @brief Recycles the tree to restart from the given node.
     * In doing so, it updates the expected information gain at each node.
     * @param node
     */
    void Tigris::recycle_tree_from_node(TreeNode *node_ptr, InfoMap &info_map)
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
        this->leaf_node_of_best_path = node_ptr; // Set starting node as best path
        // ROS_INFO_STREAM("Cost reduction is " << cost_reduction);
        // ROS_INFO_STREAM("New best path info gain is " << this->leaf_node_of_best_path->information << " and has cost " << leaf_node_of_best_path->cost.value());
        this->add_children_to_tree(info_map, node_ptr);
        ROS_INFO_STREAM("Size of recycled tree is " << this->nn_data_structure.size());
    }

    /**
     * @brief Deletes all nodes before the given node_ptr and their children without deleting the first node_ptr
     * 
     * @param node_ptr 
     */
    void Tigris::erase_nodes_before_start(TreeNode *node_ptr)
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

    /**
     * @brief Deletes the node and its children
     * 
     * @param node_ptr 
     */
    void Tigris::erase_node_and_children(TreeNode *node_ptr)
    {
        for (auto *child : node_ptr->children)
        {
            erase_node_and_children(child);
        }
        delete node_ptr;
    }

    /**
     * @brief First calls this->sample_node() to get a new node without any constraints.
     * Then extends the tree towards that node.
     * The resulting node is called the feasible_sampled_node because it's constrained to what's
     * feasible given the budget.
     *
     * @param info_map
     * @return TreeNode
     */
    TreeNode* Tigris::sample_node_within_budget(InfoMap &info_map, double budget)
    {
        try
        {
            TreeNode sample_node = this->sample_node(info_map);
            assert (this->XYZPsi_Space->satisfiesBounds(sample_node.state));
            // ROS_INFO_STREAM("Getting nearest node in tree to sampled node");
            TreeNode *nearest_node = this->nn_data_structure.nearest(&sample_node);
            // Returns nullptr if: (1) trochoid path fails
            TreeNode *feasible_sampled_node = this->extend_from_start_node_toward_goal_node(info_map, nearest_node, &sample_node, this->extend_dist);
            
            return feasible_sampled_node;
        }
        catch (std::runtime_error &e)
        {
            return nullptr;
        }
    }

    /**
     * @brief Use the InfoMap to directly sample a location from the map, and turn it into a node.
     *
     * @return TreeNode nullptr if invalid sample,
     */
    TreeNode Tigris::sample_node(InfoMap &info_map)
    {
        auto xy = info_map.sample_xy(*(this->tree_root));
        double x = xy.first, y = xy.second;

        if (isnan(x) || isnan(y))
        {
            throw std::runtime_error("Invalid sample");
        }

        ob::ScopedState<XYZPsiStateSpace> new_state(this->XYZPsi_Space);
        // Sample the altitude and angle
        double altitude = Tigris::flight_height + 5 * distribution_z(gen);
        double psi = this->distribution_psi(gen);

        // Solve for the x, y location at the altitude and angle
        double declination = this->sensor_params.pitch + this->viewpoint_goal * this->sensor_params.get_vfov() / 2; // Add because pitch is from forward direction.
        double delta_dist = altitude / tan(declination);
        x -= delta_dist * cos(psi);
        y -= delta_dist * sin(psi);

        // Set the values in the new node
        new_state->setX(x);
        new_state->setY(y);
        new_state->setZ(altitude);
        new_state->setPsi(psi);
        this->XYZPsi_Space->enforceBounds(new_state.get()); // THIS IS IMPORTANT. Clamps to [-pi, pi), needed for OMPL distance checking
        this->xyzi_samples.push_back(std::vector<double>({new_state->getX(), new_state->getY(), new_state->getZ(), 1})); // xyz and information


        ROS_DEBUG_STREAM("Sampled node at X::" << x << " Y::" << y << " Z::" << altitude << " Psi::" << psi);

        TreeNode sampled_node(this->XYZPsi_Space);

        // copy the new state into the node
        this->XYZPsi_Space->getStateSpace()->copyState(sampled_node.state, new_state.get());

        return sampled_node;
    }

    // Returns true if node was added to tree. Return false if not and should be deleted
    bool Tigris::add_node_to_tree_or_prune(InfoMap &info_map, TreeNode *add_node, TreeNode *add_node_parent)
    {
        if (add_node != nullptr)
        {
            // ROS_DEBUG_STREAM("Adding node to tree of depth " << add_node->depth);
            add_node->information = info_map.calc_child_to_root_value(*add_node, include_edge);
            if (isnan(add_node->information))
            {
                ROS_WARN_STREAM("Information is nan");
            }
            // ROS_DEBUG_STREAM("Added node with info gain " << add_node->information);

            // only add if don't prune
            if (!should_prune(add_node, this->prune_radius))
            {
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
                return true;
            }
            else
            {
                // delete add_node; // delete the node because it is pruned
                return false;
            }
        }
        ROS_ERROR("This would have been true before.");
        return false;
    }

    void print_two_nodes(XYZPsiStateSpace::StateType *node1, XYZPsiStateSpace::StateType *node2, std::string node1_name, std::string node2_name)
    {
        ROS_INFO_STREAM("Node " << node1_name << " x: " << node1->getX() <<
                        " y: " << node1->getY() <<
                        " z: " << node1->getZ() <<
                        " psi: " << node1->getPsi());
        ROS_INFO_STREAM("Node " << node2_name << " x: " << node2->getX() <<
                        " y: " << node2->getY() <<
                        " z: " << node2->getZ() <<
                        " psi: " << node2->getPsi());
    }

    /**
     * @brief Extends neighbors on the tree towards the given node.
     */
    void Tigris::extend_neighbors_towards_node(InfoMap &info_map, TreeNode *node_feasible, double budget)
    {
        // First add the feasible node in cases where extend_radius is smaller than extend_dist
        bool save_node_feasible = false;
        if (node_feasible->incremental_cost.value() > 0.01)
        {
            save_node_feasible = this->add_node_to_tree_or_prune(info_map, node_feasible, node_feasible->parent);
        }
        else
        {
            // ROS_DEBUG_STREAM("Node feasible has incremental cost of " << node_feasible->incremental_cost.value());
            // print_two_nodes(node_feasible->state->as<XYZPsiStateSpace::StateType>(), 
            //                 node_feasible->parent->state->as<XYZPsiStateSpace::StateType>(), 
            //                 "Node Feasible", "Node Feasible parent");
        }

        // ROS_INFO("Extending toward feasible node");
        
        // ROS_INFO_STREAM("Getting nearest neighbors");
        // Find all other near points to be extended toward new point
        std::vector<TreeNode *> near_neighbors;
        this->nn_data_structure.nearestR(node_feasible, this->extend_radius, near_neighbors);
   
        for (auto *node_near : near_neighbors)
        {
            // Skip self node, parent node, and nodes in closed set
            if (node_near == node_feasible && !save_node_feasible)
                ROS_ERROR_STREAM("This shouldn't Happen"); 
            
            if (node_near != node_feasible && node_near != node_feasible->parent && !node_near->is_in_closed_set)
            {
                if (node_near->is_in_closed_set){
                    ROS_ERROR("It messed up");
                }
                if (node_near == node_feasible->parent){
                    ROS_ERROR("It messed up again"); // This
                }
                // Eigen::Vector3d diff = node_near->state->as<XYZPsiStateSpace::StateType>()->getXYZ() - node_feasible->state->as<XYZPsiStateSpace::StateType>()->getXYZ();
                // if (XYZPsi_Space->distance(node_near->state, node_feasible->state) > TOLERANCE && diff.norm() < TOLERANCE)
                // {
                //     ROS_ERROR_STREAM("Nodes are colocated but different psi");
                //     print_two_nodes(node_near->state->as<XYZPsiStateSpace::StateType>(), node_feasible->state->as<XYZPsiStateSpace::StateType>(), "Node Near", "Node Feasible");
                // }
                if (XYZPsi_Space->distance(node_near->state, node_feasible->state) < TOLERANCE)
                {
                    // ROS_DEBUG("Nodes colocated. No need to extend. Skip");
                    continue;
                }

                // TODO: we should convert all the pointers to unique_ptrs
                // Returns nullptr if: (1) trochoid path fails
                auto *add_node = this->extend_from_start_node_toward_goal_node(info_map, node_near, node_feasible, this->extend_dist);
                if (add_node == nullptr)
                {
                    ROS_WARN_STREAM("Trochoid path failed. Skipping");
                    print_two_nodes(node_near->state->as<XYZPsiStateSpace::StateType>(), 
                                    node_feasible->state->as<XYZPsiStateSpace::StateType>(), 
                                    "Node Near", "Node Feasible");
                    continue;
                }

                if (add_node->incremental_cost.value() < 0.01)
                {
                    // ROS_INFO("Near node was too close to the feasible. Skipping");
                    // print_two_nodes(node_near->state->as<XYZPsiStateSpace::StateType>(), 
                    //                 add_node->state->as<XYZPsiStateSpace::StateType>(), 
                    //                 "Node Near", "Add Node");
                    // ROS_INFO_STREAM("Near node cost: " << node_near->cost.value() << " add node cost: " << add_node->cost.value());
                    delete add_node;
                    continue;
                }
                if (abs(node_near->cost.value() - node_feasible->parent->cost.value()) < .001 && abs(node_near->information - node_feasible->parent->information) < .001){
                    ROS_INFO_STREAM("Near node and feasible parent have same cost and info");
                    ROS_INFO_STREAM("Near cost: " << node_near->cost.value() << 
                                     " info: " << node_near->information <<
                                     " depth: " << node_near->depth <<
                                     " budget remaining: " << node_near->budget_remaining);
                    ROS_INFO_STREAM("Feasible parent cost: " << node_feasible->parent->cost.value() <<
                                        " info: " << node_feasible->parent->information <<
                                        " depth: " << node_feasible->parent->depth <<
                                        " budget remaining: " << node_feasible->parent->budget_remaining);
                    // if (add_node->incremental_cost.value() < 0.01)
                    //     ROS_ERROR("And the new one is too close");
                    // else
                    //     ROS_ERROR("Same but not close");
                }


                if (!this->add_node_to_tree_or_prune(info_map, add_node, node_near))
                {
                    delete add_node;
                }
            }
        }
        if (!save_node_feasible)
        {
            // ROS_INFO("Deleting feasible node");
            delete node_feasible;
        }
        // std::cout << "Tree size: " << this->nn_data_structure.size() << std::endl;
    }

    std::string Tigris::get_plan_metrics() 
    {
        int num_waypoints = 0;
        std::vector<og::PathGeometric> best_path_segments = this->get_best_path_segments();
        for (auto& path_segment : best_path_segments) {
            for (int i = 1; i < path_segment.getStateCount(); ++i) {
                num_waypoints += 1;
            } 
        }
        std::stringstream plan_metrics_str;
        plan_metrics_str << this->leaf_node_of_best_path->information << ","
                         << this->leaf_node_of_best_path->cost.value() << ","
                         << num_waypoints << ","
                         << best_path_segments.size() << ","
                         << this->num_samples_drawn << ","
                         << this->nn_data_structure.size();
        return plan_metrics_str.str();
    }

    // TODO: this assumes modular cost. Need to update later
    bool Tigris::should_prune(TreeNode *new_node, double radius) const
    {
        std::vector<TreeNode *> near_neighbors;
        // Find all other near points to compare agains
        this->nn_data_structure.nearestR(new_node, radius, near_neighbors);
        for (auto node_near : near_neighbors)
        {
            if (node_near->cost.value() <= new_node->cost.value() && node_near->information >= new_node->information)
            {
                ROS_DEBUG_STREAM(
                    "Pruned node because neighbor cost " << node_near->cost.value() << " <= new node cost " << new_node->cost.value() 
                    << " && neighbor info " << node_near->information << " >= new node info " << new_node->information);
                return true;
            }
        }
        return false;
    }

    // Adds the children of the parent to the tree with updated information and cost
    void Tigris::add_children_to_tree(InfoMap &info_map, TreeNode *parent)
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
            Tigris::add_children_to_tree(info_map, child);
        }
    }

    // Deletes the node and its children
    void Tigris::delete_node_and_children(TreeNode *parent)
    {
        for (auto *child : parent->children)
        {
            delete_node_and_children(child);
        }
        delete parent;
    }

    /**
     * @brief Return read-only pointers of all the nodes of the tree in a vector.
     * 
     * @return const std::vector<TreeNode *>& 
     */
    const std::vector<TreeNode *> Tigris::get_all_nodes() const
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


    TreeNode *Tigris::get_leaf_node_of_best_path() const
    {
        return leaf_node_of_best_path;
    }; // TODO return the path and not the pointer

    // return the xy_samples vector
    const std::vector<std::vector<double>> &Tigris::get_sampled_xyzi() const
    {
        return xyzi_samples;
    };

} // namespace ipp
