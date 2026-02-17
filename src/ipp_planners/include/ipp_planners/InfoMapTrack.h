
#ifndef INFORMATIVE_BELIEF_TRACK_H
#define INFORMATIVE_BELIEF_TRACK_H
#include <vector>
#include <utility>
#include <unordered_set>

#include <planner_map_interfaces/TargetPrior.h>

#include "ipp_planners/InfoMap.h"
#include "ipp_planners/TreeNode.h"
#include "ipp_belief/belief_manager.h"
#include "ipp_belief/information.h"

#include <ipp_belief/ClearTargetPriors.h>
#include <ipp_belief/AddTargetPriors.h>
#include <ipp_belief/RemoveTargetPriors.h>
#include "core_planning_state_space/state_spaces/xyzpsi_state_space.h"
#include "core_planning_state_space/state_space_utils/xyzpsi_state_space_utils.h"

namespace ipp
{
    /**
     * @brief Represents a sensor footprint. A drone pose, 4 points of a quadilateral
     * footprint, and a delta time to take place before this footprint happens
     *
     */
    struct Footprint
    {
        std::vector<double> drone_pose;
        std::vector<double> p1;
        std::vector<double> p2;
        std::vector<double> p3;
        std::vector<double> p4;
        double delta_t;
    };

    bool is_turn(XYZPsiStateSpace::StateType *state1, XYZPsiStateSpace::StateType *state2, double eps = 0.0001)
    {
        return abs(state1->getPsi() - state2->getPsi()) > eps;
    }

    /**
     * @brief This function returns the points of the camera projection on the plane
     *
     * @param agent_pos XYZPsi
     * @param q_rotated
     * @return Footprint
     */
    Footprint project_camera_bounds_to_plane(std::vector<double> agent_pos,
                                                    std::vector<std::vector<double>> q_rotated)
    {
        std::vector<std::vector<double>> projected_camera_bounds;
        for (int i = 0; i < q_rotated.size(); ++i)
        {
            double translated_x = agent_pos[0] + q_rotated[i][0];
            double translated_y = agent_pos[1] + q_rotated[i][1];
            double translated_z = agent_pos[2] + q_rotated[i][2];

            double x = agent_pos[0] -
                       (agent_pos[0] - translated_x) *
                           (-agent_pos[2] / (agent_pos[2] - translated_z));
            double y = agent_pos[1] -
                       (agent_pos[1] - translated_y) *
                           (-agent_pos[2] / (agent_pos[2] - translated_z));
            double z = agent_pos[2] +
                       (agent_pos[2] - translated_z) *
                           (-agent_pos[2] / (agent_pos[2] - translated_z));

            std::vector<double> intercept = {x, y, z};
            projected_camera_bounds.push_back(intercept);
        }
        Footprint f = {agent_pos, projected_camera_bounds[0], projected_camera_bounds[1],
                       projected_camera_bounds[2], projected_camera_bounds[3]};
        assert(f.drone_pose[0] == agent_pos[0]);
        assert(f.drone_pose[1] == agent_pos[1]);
        assert(f.drone_pose[2] == agent_pos[2]);
        return f;
    }

    /**
     * @brief turns a state type into an observatino type
     *
     */
    tracking::Observation make_observation_from_state(XYZPsiStateSpace::StateType *state, SensorParams sensor_params)
    {
        std::vector<std::vector<double>> q_rotated = rotated_camera_fov(
            sensor_params,
            /*roll*/ 0.0,
            /*pitch*/ sensor_params.pitch,
            /*yaw*/ state->getPsi());
        Footprint f = project_camera_bounds_to_plane(
            {state->getX(), state->getY(), state->getZ()}, q_rotated);

        // make a polygonal observation from the points.
        std::vector<double> obs_points; // x_0, y_0, x_1, y_1, ...
        obs_points.insert(obs_points.end(), f.p1.begin(), f.p1.end());
        obs_points.insert(obs_points.end(), f.p2.begin(), f.p2.end());
        obs_points.insert(obs_points.end(), f.p3.begin(), f.p3.end());
        obs_points.insert(obs_points.end(), f.p4.begin(), f.p4.end());
        tracking::Polygon obs_region(obs_points);
        tracking::TargetState drone_pose(0, state->getX(), state->getY(), state->getZ(), state->getPsi());
        tracking::Observation observation(obs_region, drone_pose, sensor_params);
        return observation;
    }

    /**
     * @brief Discretizes observations along the trochoid path from the parent node to the child node.
     * Traces the path from the child node to the parent node.
     *
     * @param child node
     * @param max_num_parents how many parents to get
     * @return std::pair<std::vector<tracking::Observation>, std::vector<double>>
     */
    std::pair<std::vector<tracking::Observation>, std::vector<double>>
    discretize_observations(
        TreeNode &child_node, 
        double desired_speed, 
        SensorParams sensor_params, 
        int max_num_parents = std::numeric_limits<int>::max(), 
        double straight_discretization_distance = 400
        )
    {
        BOOST_ASSERT_MSG(desired_speed > 0, "desired speed must be positive");
        std::vector<tracking::Observation> observations;
        std::vector<double> time_deltas;

        std::list<TreeNode *> path = child_node.get_path_from_parent(max_num_parents);

        auto euclidean_distance = [](XYZPsiStateSpace::StateType *state1, XYZPsiStateSpace::StateType *state2)
        {
            Eigen::Vector3d diff = state1->getXYZ() - state2->getXYZ();
            return diff.norm();
        };

        double accumulated_distance = 0;

        for (auto node : path)
        {
            auto path_states = node->edge_trochoid.getStates();
            double dist_from_last_observation = 0;
            for (auto it = path_states.begin() + 1; it != path_states.end(); it++)
            {
                auto prev_state = (*std::prev(it))->as<XYZPsiStateSpace::StateType>();
                auto current_state = (*it)->as<XYZPsiStateSpace::StateType>();
                // even though we have turns, this euclidean distance is a close approximation because the trochoid path is very fine
                dist_from_last_observation += euclidean_distance(prev_state, current_state);

                // make turns discretize x3 more than straight lines. TODO: tune parameters based on sensor footprint size and turning rate
                double discretization_dist = is_turn(prev_state, current_state) ? straight_discretization_distance / 5.5 : straight_discretization_distance;

                if (dist_from_last_observation >= discretization_dist)
                {
                    // make an observation
                    accumulated_distance += dist_from_last_observation;
                    tracking::Observation obs = make_observation_from_state(current_state, sensor_params);
                    // calculate the time delta
                    double time_delta = dist_from_last_observation / desired_speed;
                    // save
                    observations.push_back(obs);
                    time_deltas.push_back(time_delta);
                    // continue
                    dist_from_last_observation = 0;
                }
            }
        }
        ROS_DEBUG_STREAM("accumulated distance: " << accumulated_distance << " child node cost: " << child_node.cost.value());
        assert (accumulated_distance <= child_node.cost.value() + 5); // some room for error
        // add at least one node, cuz otherwise it breaks
        if (observations.size() == 0){
            observations.push_back(make_observation_from_state(child_node.state->as<XYZPsiStateSpace::StateType>(), sensor_params));
            time_deltas.push_back(0);
        }
        return std::pair(observations, time_deltas);
    }

    /**
     * @brief Perform target tracking objective. Samples based on belief particles
     *
     */
    class InfoMapTrack : public InfoMap
    {
    protected:
        // index of the tracker we sampled last. for round robin sampling
        int last_sampled_tracker_id;
        double variance_cap;
        std::unordered_map<unsigned int, double> id_to_weights_that_decrease_sampling_prev_tracker;

    public:
        // representation of the target tracks
        tracking::ParticleFiltersBelief particle_belief_manager;

        explicit InfoMapTrack(ros::NodeHandle &nh, ros::NodeHandle &pnh)
            : InfoMap(nh,pnh),
              last_sampled_tracker_id(0)
        {
            this->desired_speed = -1; // overwritten later with actual value from planning request
            this->variance_cap = ros_utils::get_param<double>(this->nh, "ipp_belief/variance_cap");
        }

        ~InfoMapTrack() = default;

        void copy_params_from(const InfoMapTrack &other)
        {
            this->bounds = other.bounds;
            this->sensor_params = other.sensor_params;
            this->desired_speed = other.desired_speed;
            this->particle_belief_manager = other.particle_belief_manager;
            this->last_sampled_tracker_id = other.last_sampled_tracker_id;
            this->id_to_weights_that_decrease_sampling_prev_tracker = other.id_to_weights_that_decrease_sampling_prev_tracker;
        }

        // get particle belief manager
        const tracking::ParticleFiltersBelief &get_particle_belief_manager()
        {
            return particle_belief_manager;
        }

        // set particle belief manager
        void set_particle_belief_manager(tracking::ParticleFiltersBelief &particle_belief_manager)
        {
            this->particle_belief_manager = particle_belief_manager;
            for (auto tracker_id : particle_belief_manager.get_tracker_ids())
            {
                id_to_weights_that_decrease_sampling_prev_tracker.insert({tracker_id, 1.0});
            }
        }

        void reset_sampling_state()
        {
            last_sampled_tracker_id = 0;
        }

        /* =============================
         * ---- OVERRIDEN METHODS ----
         * ============================= */

        bool send_plan_request_params_to_belief(const planner_map_interfaces::PlanRequest &msg, double flight_height = 80) override
        {
            ipp_belief::AddTargetPriors srv;
            // TODO: this is a temporary fix to not track target priors lacking an initialized FilteredTarget.
            // ideally we should factor out polygon priors and filtered targets
            for (auto target_prior : msg.target_priors)
            {
                if (!is_target_uninitialized(target_prior.target))
                {
                    srv.request.target_priors.push_back(target_prior);
                }
            }
            srv.request.target_priors = msg.target_priors;
            if (ros::service::call("ipp_belief/add_target_priors_srv", srv))
            {
                return true;
            }
            return false;
        }

        bool fetch_latest_belief(const std::vector<double> &pose_to_plan_from, const double &planning_budget) override
        {
            // wait to make sure to get the updated copy of the belief
            ROS_INFO("Fetching latest track info");
            auto new_particle_filters_belief_msg = ros::topic::waitForMessage<ipp_belief::ParticleFiltersBelief>("ipp_belief/particle_filters_belief", ros::Duration(10));
            if (new_particle_filters_belief_msg == NULL)
            {
                ROS_ERROR("Did not receive updated tracking belief from replan request");
                return false;
            }

            auto pbm = tracking::belief_manager_from_ros_msg(new_particle_filters_belief_msg);
            this->set_particle_belief_manager(pbm);
            return true;
        }

        /**
         * @brief Samples a particle from the belief particles.
         *
         * @param node the node that the drone is currently at
         * @return std::vector<double>
         */
        std::pair<double, double> sample_xy(const TreeNode &node) override
        {
            if (particle_belief_manager.get_num_active_trackers() == 0)
            {
                ROS_WARN(
                    "No trackers in belief manager! There is no "
                    "information to be had. Not moving");
                auto *s_ = node.state->as<XYZPsiStateSpace::StateType>();
                return std::make_pair(s_->getX(), s_->getY());
            }
            auto &tracker = this->sample_tracker(node);
            if (tracker.get_num_particles() == 0)
            {
                ROS_WARN(
                    "No particles in tracker! There is no "
                    "information to be had. Not moving");
                auto *s_ = node.state->as<XYZPsiStateSpace::StateType>();
                return std::make_pair(s_->getX(), s_->getY());
            }
            auto particle = tracker.sample_particle_weighted();

            auto *node_state = node.state->as<XYZPsiStateSpace::StateType>();
            auto drone_x = node_state->getX();
            auto drone_y = node_state->getY();

            // auto xy = dumb_not_really_intersection(drone_x, drone_y, this->desired_speed, particle);
            auto xy = solve_soonest_intersection_drone_to_particle(drone_x, drone_y, this->desired_speed, particle);

            return xy;
        }

        /**
         * @brief Check if collided with any targets
         * 
         * @param state 
         * @return true 
         * @return false 
         */
        bool is_collision(XYZPsiStateSpace::StateType *state) override{
            if (this->counter_detect_radius > 0.01) // check if care about collision checking
            {
                double x = state->getX();
                double y = state->getY();
                for (auto &[id, tracker]: this->particle_belief_manager.get_id_to_trackers()){
                    auto mean = tracker.get_mean_particle();
                    if (sqrt(pow(x - mean.get_x(), 2.0) + pow(y - mean.get_y(), 2.0)) <= this->counter_detect_radius){
                        return true;
                    }
                }
            }
            return false;
        }

        /**
         * @brief Calculates the information value of the path from the child node to the root.
         * If the child node stops has remaining budget, then continues to propagate the information map for the duration of staying still.
         *
         * @param child_node
         * @param is_edge_included
         * @return [double, ParticleFiltersBelief] The positive information value and the belief at the end of the path
         */
        double calc_child_to_root_value(TreeNode &child_node, const bool is_edge_included = true) override
        {
            double total_time = (child_node.cost.value() + child_node.budget_remaining) / this->desired_speed;
            ROS_DEBUG_STREAM("Total time: " << total_time);
            if (total_time < 0.0001)
            {
                return 0;
            }
            auto [observations, time_deltas] = discretize_observations(
                child_node, this->desired_speed, this->sensor_params, std::numeric_limits<int>::max(), this->observation_discretization_distance
            );
            double accumulated_time_at_node = std::accumulate(time_deltas.begin(), time_deltas.end(), 0.0);
            ROS_DEBUG_STREAM("Accumulated discretized time at node: " << accumulated_time_at_node);
            // this is important to make TIGRIS work. otherwise tigris is like "no i don't want to go anywhere"
            double time_of_staying_still = child_node.budget_remaining / this->desired_speed;
            observations.push_back(observations.back());
            time_deltas.push_back(time_of_staying_still);
            double total_time_from_deltas = std::accumulate(time_deltas.begin(), time_deltas.end(), 0.0);
            ROS_DEBUG_STREAM("Total time from deltas: " << total_time_from_deltas);

            // ROS_DEBUG_STREAM("Time of staying still: " << time_of_staying_still);
            // ROS_INFO_STREAM("Size of observations: " << observations.size());

            // pass observations to `calculate_information_gain` function
            double value;
            tracking::ParticleFiltersBelief transition_belief;
            if (ros_utils::get_param<std::string>(this->pnh, "information_metric") == "particles")
            {
                auto [gain_, belief_copy] = tracking::calculate_particle_info_gain(this->particle_belief_manager, observations, time_deltas, this->variance_cap);
                value = gain_;
            }
            else if (ros_utils::get_param<std::string>(this->pnh, "information_metric") == "variance_delta")
            {
                auto [gain_, belief_copy] =
                    tracking::calculate_variance_delta(this->particle_belief_manager, observations, time_deltas, tracking::transform_variance_info_gain, this->variance_cap);
                value = gain_;
            }
            else if (ros_utils::get_param<std::string>(this->pnh, "information_metric") == "variance_accumulated")
            {
                auto [variance, belief_copy] =
                    tracking::calculate_variance_accumulated(this->particle_belief_manager, observations, time_deltas, this->variance_cap);
                value = -variance;
            }
            else
            {
                ROS_ERROR_STREAM("Unknown information metric " << ros_utils::get_param<std::string>(this->pnh, "information_metric"));
                throw std::runtime_error("Invalid information metric");
            }
            double average_value_over_time = value / total_time;
            return average_value_over_time;
        }

        std::unique_ptr<InfoMapTrack> clone()
        {
            auto new_info_map = std::make_unique<InfoMapTrack>(*this);
            return new_info_map;
        }

        /* =============================
         * ---- HELPER METHODS ----
         * ============================= */

        bool is_target_uninitialized(planner_map_interfaces::FilteredTarget &t)
        {
            bool is_cov_all_zero = true;
            for (int i = 0; i < t.covariance.size(); i++)
            {
                if (t.covariance[i] != 0)
                {
                    is_cov_all_zero = false;
                }
            }
            return is_cov_all_zero && t.x == 0 && t.y == 0 && t.xdot == 0 && t.ydot == 0;
        }


        const tracking::ParticleFilter &sample_tracker(const TreeNode &node)
        {
            if (particle_belief_manager.get_num_active_trackers() == 0)
            {
                throw std::runtime_error("No trackers in belief manager, cannot sample tracker");
            }

            return sample_tracker_uniform(); // TODO: put choice in ros param
            // return sample_tracker_round_robin();
            // return sample_tracker_decrease_sampling_previous();
            // return sample_tracker_weighted_by_distance_and_min_variance(node);
        }

        /**
         * @brief Performs a uniform sample over trackers with at least 1 particle.
         *
         * @return const tracking::ParticleFilter&
         */

        const tracking::ParticleFilter &sample_tracker_uniform()
        {
            auto valid_ship_ids = particle_belief_manager.get_tracker_ids();
            valid_ship_ids = filter_target_ids_with_more_than_one_particle(valid_ship_ids);
            valid_ship_ids = filter_target_ids_within_variance(valid_ship_ids, 10);
            if (valid_ship_ids.size() == 0)
            {
                ROS_WARN_STREAM("No valid trackers to sample from, sampling from all trackers");
                valid_ship_ids = particle_belief_manager.get_tracker_ids();
            }
            std::uniform_int_distribution<> tracker_dis(
                0, valid_ship_ids.size() - 1);
            int tracker_idx = tracker_dis(gen);
            int tracker_id = valid_ship_ids.at(tracker_idx);

            auto &tracker = particle_belief_manager.get_tracker(tracker_id);
            assert(tracker.get_num_particles() > 0);
            return tracker;
        }

        /**
         * @brief When a tracker is sampled, decrease the probability that it will be sampled again
         *
         * @param node
         * @return const tracking::ParticleFilter&
         */
        const tracking::ParticleFilter &sample_tracker_decrease_sampling_previous()
        {

            auto valid_ship_ids = particle_belief_manager.get_tracker_ids();
            valid_ship_ids = filter_target_ids_with_more_than_one_particle(valid_ship_ids);
            valid_ship_ids = filter_target_ids_within_variance(valid_ship_ids, 10);

            if (valid_ship_ids.size() == 0)
            {
                ROS_WARN_STREAM("No valid trackers to sample from, sampling from all trackers");
                valid_ship_ids = particle_belief_manager.get_tracker_ids();
            }

            std::vector<tracking::ParticleFilter> valid_trackers;
            std::vector<double> valid_weights;
            for (auto id : valid_ship_ids)
            {
                valid_trackers.push_back(particle_belief_manager.get_tracker(id));
                valid_weights.push_back(this->id_to_weights_that_decrease_sampling_prev_tracker.at(id));
            }
            std::discrete_distribution<int> distribution(valid_weights.begin(), valid_weights.end());
            int sampled_index = distribution(gen);
            auto sampled_ship_id = valid_ship_ids[sampled_index];

            // update weights
            this->id_to_weights_that_decrease_sampling_prev_tracker.at(sampled_ship_id) *= 1.0 / particle_belief_manager.get_num_trackers();
            double weight_sum = std::accumulate(valid_weights.begin(), valid_weights.end(), 0.0);
            // normalize the weights
            for (auto id : valid_ship_ids)
            {
                this->id_to_weights_that_decrease_sampling_prev_tracker.at(id) /= weight_sum;
            }

            ROS_DEBUG_STREAM("Sampling tracker with id: " << sampled_ship_id);
            return particle_belief_manager.get_tracker(sampled_ship_id);
        }

        /**
         * @brief Samples over trackers in a round-robin fashion, to ensure that each tracker gets a chance to be sampled.
         *
         * @return const tracking::ParticleFilter&
         */
        const tracking::ParticleFilter &sample_tracker_round_robin()
        {
            auto valid_ship_ids = filter_target_ids_with_more_than_one_particle(particle_belief_manager.get_tracker_ids());
            valid_ship_ids = filter_target_ids_within_variance(valid_ship_ids, 10);
            if (valid_ship_ids.size() == 0)
            {
                ROS_WARN_STREAM("No valid trackers to sample from, sampling from all trackers");
                valid_ship_ids = particle_belief_manager.get_tracker_ids();
            }

            std::sort(valid_ship_ids.begin(), valid_ship_ids.end());
            // for (auto valid_ship_id: valid_ship_ids){
            //     std::cout << "valid_ship_id: " << valid_ship_id << std::endl;
            // }
            int idx = 0;
            while (valid_ship_ids[idx] <= this->last_sampled_tracker_id)
            {
                ++idx;
            }
            if (idx >= valid_ship_ids.size())
            {
                idx = 0;
            }

            unsigned int tracker_id = valid_ship_ids[idx];
            this->last_sampled_tracker_id = tracker_id;
            ROS_DEBUG_STREAM("Sampling tracker with id: " << tracker_id);
            auto &tracker = particle_belief_manager.get_tracker(tracker_id);
            return tracker;
        }

        /**
         * @brief weights trackers by their relative distance, closer is better
         *
         * @param node
         * @return const tracking::ParticleFilter&
         */
        const tracking::ParticleFilter &sample_tracker_weighted_by_distance_and_min_variance(const TreeNode &node)
        {
            auto *s_ = node.state->as<XYZPsiStateSpace::StateType>();
            double x = s_->getX(), y = s_->getY();

            auto valid_ship_ids = filter_target_ids_with_more_than_one_particle(particle_belief_manager.get_tracker_ids());
            std::vector<double> distances_of_valid_ships;
            for (auto ship_id : valid_ship_ids)
            {
                auto mean_particle = particle_belief_manager.get_tracker(ship_id).get_mean_particle();
                double particle_x = mean_particle.get_x(), particle_y = mean_particle.get_y();
                double distance = std::sqrt(std::pow(x - particle_x, 2) + std::pow(y - particle_y, 2));
                distances_of_valid_ships.push_back(distance);
            }
            double max_distance = *std::max_element(distances_of_valid_ships.begin(), distances_of_valid_ships.end());

            std::vector<double> target_variances;
            for (auto ship_id : valid_ship_ids)
            {
                auto tracker = particle_belief_manager.get_tracker(ship_id);
                double variance = tracking::calculate_trace_variance(tracker);
                target_variances.push_back(variance);
            }

            std::vector<double> weights;
            for (int i = 0; i < valid_ship_ids.size(); i++)
            {
                double distance = distances_of_valid_ships[i];
                double variance = target_variances[i];
                double weight = (max_distance * 1.25 - distance) / max_distance;
                if (variance < 20)
                    weight *= std::pow(variance / 20, 2);
                weights.push_back(weight);
            }

            // sample based on the weight
            std::discrete_distribution<int> distribution(weights.begin(), weights.end());
            int sampled_index = distribution(gen);
            auto sampled_ship_id = valid_ship_ids[sampled_index];

            return particle_belief_manager.get_tracker(sampled_ship_id);
        }

        /**
         * @brief There is only uncertainty if there is more than 1 particle
         *
         * @param prior_ids
         * @return std::vector<unsigned int>
         */
        std::vector<unsigned int> filter_target_ids_with_more_than_one_particle(std::vector<unsigned int> prior_ids)
        {
            std::vector<unsigned int> valid_ship_ids;
            // make sure the trackers we sample from have at least one particle
            std::copy_if(prior_ids.begin(), prior_ids.end(), std::back_inserter(valid_ship_ids),
                         [this](unsigned int target_id)
                         { return this->particle_belief_manager.get_tracker(target_id).get_num_particles() > 1; });
            return valid_ship_ids;
        }

        std::vector<unsigned int> filter_target_ids_within_variance(std::vector<unsigned int> prior_ids, double min_variance = 10, double max_variance=100000)
        {
            std::vector<unsigned int> valid_ship_ids;
            // make sure the trackers we sample from have at least one particle
            std::copy_if(prior_ids.begin(), prior_ids.end(), std::back_inserter(valid_ship_ids),
                         [this, min_variance, max_variance](unsigned int target_id)
                         { return min_variance < tracking::calculate_trace_variance(this->particle_belief_manager.get_tracker(target_id)) < max_variance; });
            return valid_ship_ids;
        }

        int get_num_tracks()
        {
            return particle_belief_manager.get_num_trackers();
        }

        /**
         * @brief Naive method that propagates the particle forward by the time it takes for the drone to traverse the tree to that point.
         *
         * @param drone_x
         * @param drone_y
         * @param drone_speed
         * @param particle
         * @return std::pair<double, double>
         */
        std::pair<double, double> dumb_not_really_intersection(double drone_x, double drone_y, double drone_speed, const tracking::TargetState &particle)
        {
            // now propagate that sample by min time it takes to fly there
            double dist_root_to_particle = std::sqrt(std::pow(drone_x - particle.get_x(), 2) + std::pow(drone_y - particle.get_y(), 2));
            double min_flight_time = (dist_root_to_particle) / this->desired_speed;
            double dt = min_flight_time * 1.0;

            double x = particle.get_x() + std::cos(particle.get_heading()) * (particle.get_speed() * dt);
            double y = particle.get_y() + std::sin(particle.get_heading()) * (particle.get_speed() * dt);

            return std::make_pair(x, y);
        }

        /**
         * @brief find the xy location of the soonest theoretic intersection that the drone could make to the particle.
         * TODO: take into account drone heading and motion model. this is a 20/80 solution
         * https://stackoverflow.com/a/22117046/5118517
         * @param drone_x
         * @param drone_y
         * @param drone_speed
         * @param particle
         * @return std::pair<double, double>  returns <nan, nan> if no solution. i.e. particle faster than drone
         */
        static std::pair<double, double> solve_soonest_intersection_drone_to_particle(double drone_x, double drone_y, double drone_speed, const tracking::TargetState &particle)
        {
            // 0 = particle position. 1 = drone position.
            double P0x = particle.get_x(), P0y = particle.get_y();
            double s0 = particle.get_speed();
            double V0x = std::cos(particle.get_heading());
            double V0y = std::sin(particle.get_heading());

            double P1x = drone_x, P1y = drone_y;
            double s1 = drone_speed;

            // quadratic formula
            double a = (s0 * s0) - (s1 * s1);
            // if (abs((V0x * V0x) + (V0y * V0y) - 1) > 0.000001)
            // {
            //     ROS_ERROR_STREAM("Squared and summed not zero?: " << ((V0x * V0x) + (V0y * V0y)));
            // }
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
                    << " particle_x=" << particle.get_x() << " particle_y=" << particle.get_y()
                    << " particle_speed=" << particle.get_speed() << " particle_heading=" << particle.get_heading());
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
    };

}
#endif // INFORMATIVE_BELIEF_TRACK_H
