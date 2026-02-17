#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "trackers.h"
#include "visualize.h"
#include <ipp_belief/ParticleFiltersBelief.h> // include custom ROS messages

namespace tracking
{
    TargetGaussian make_target_gaussian_from_filtered_target(
        const planner_map_interfaces::FilteredTarget &f_target)
    {
        double heading = std::atan2(f_target.ydot, f_target.xdot);
        double linear_speed = std::sqrt(f_target.xdot * f_target.xdot + f_target.ydot * f_target.ydot);
        // linear_speed *= 0.05;                           // tune hack
        linear_speed = std::max(linear_speed, 1e-4);    // avoid divide by zero ; we don't need a huge amount of heading
        double xdot = linear_speed * std::cos(heading); // safe xdot if linear speed was zero
        double ydot = linear_speed * std::sin(heading); // safe ydot if linear speed was zero

        double z = 0;
        double angular_speed = 0;
        TargetState means(f_target.global_id, f_target.x, f_target.y, z, heading, linear_speed, angular_speed);

        Eigen::Matrix<double, 4, 4> raw_cov(f_target.covariance.data());
        // scale up the raw covariance, because we don't trust the kalman filter much
        raw_cov(0, 0) *= 1;
        raw_cov(1, 1) *= 1;
        raw_cov(2, 2) *= 1;
        raw_cov(3, 3) *= 1;
        Eigen::Matrix4d jacobian; // ∂x,y,heading,linear_speed / ∂x,y,xdot,ydot
        jacobian << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, -ydot / (xdot * xdot + ydot * ydot), xdot / (xdot * xdot + ydot * ydot),
            0, 0, xdot / linear_speed, ydot / linear_speed;

        Eigen::Matrix4d result = jacobian * raw_cov * jacobian.transpose();
        // set to identity
        Eigen::Matrix<double, 5, 5> target_state_cov = Eigen::Matrix<double, 5, 5>::Identity();
        target_state_cov.block(0, 0, 4, 4) = result;
        target_state_cov(4, 4) = 0.001; // angular speed var
        // ROS_DEBUG_STREAM("means: " << means);
        // ROS_DEBUG_STREAM("raw_cov: " << raw_cov);
        // ROS_DEBUG_STREAM("transformed target_state_cov: " << target_state_cov);
        TargetGaussian target_gaussian(means, target_state_cov);
        return target_gaussian;
    }

    TargetGaussian make_target_gaussian_from_fake_observed_target(tracking::TargetState mean_state)
    {
        double x = mean_state.get_x();
        double y = mean_state.get_y();
        double heading = mean_state.get_heading();
        double linear_speed = mean_state.get_speed();
        // linear_speed *= 0.05;                           // tune hack
        linear_speed = std::max(linear_speed, 1e-4);    // avoid divide by zero ; we don't need a huge amount of heading
        double xdot = linear_speed * std::cos(heading); // safe xdot if linear speed was zero
        double ydot = linear_speed * std::sin(heading); // safe ydot if linear speed was zero

        double z = 0;
        double angular_speed = 0;
        TargetState means(mean_state.get_id(), x, y, z, heading, linear_speed, angular_speed);

        Eigen::Matrix<double, 4, 4> raw_cov;
        raw_cov << 100, 0, 0, 0,
                    0, 100, 0, 0,
                    0, 0, 0.01, 0,
                    0, 0, 0, 0.01;
        // scale up the raw covariance, because we don't trust the kalman filter much
        raw_cov(0, 0) *= 1;
        raw_cov(1, 1) *= 1;
        raw_cov(2, 2) *= 1;
        raw_cov(3, 3) *= 1;
        Eigen::Matrix4d jacobian; // ∂x,y,heading,linear_speed / ∂x,y,xdot,ydot
        jacobian << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, -ydot / (xdot * xdot + ydot * ydot), xdot / (xdot * xdot + ydot * ydot),
            0, 0, xdot / linear_speed, ydot / linear_speed;

        Eigen::Matrix4d result = jacobian * raw_cov * jacobian.transpose();
        // set to identity
        Eigen::Matrix<double, 5, 5> target_state_cov = Eigen::Matrix<double, 5, 5>::Identity();
        target_state_cov.block(0, 0, 4, 4) = result;
        target_state_cov(4, 4) = 0.001; // angular speed var
        // ROS_DEBUG_STREAM("means: " << means);
        // ROS_DEBUG_STREAM("raw_cov: " << raw_cov);
        // ROS_DEBUG_STREAM("transformed target_state_cov: " << target_state_cov);
        TargetGaussian target_gaussian(means, target_state_cov);
        return target_gaussian;
    }

    std::unordered_map<unsigned int, LinearAngularController> parse_class_controls(XmlRpc::XmlRpcValue class_controls)
    {
        std::unordered_map<unsigned int, LinearAngularController> class_id_to_controller;
        for (int i = 0; i < class_controls.size(); i++)
        {
            XmlRpc::XmlRpcValue class_control = class_controls[i];
            std::string class_label = class_control["class_label"];
            ROS_ASSERT(class_control["class_id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
            unsigned int class_id = static_cast<int>(class_control["class_id"]);
            ROS_ASSERT(class_control["linear_acceleration_mean"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            double linear_acceleration_mean = static_cast<double>(class_control["linear_acceleration_mean"]);
            ROS_ASSERT(class_control["angular_velocity_mean"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            double angular_velocity_mean = static_cast<double>(class_control["angular_velocity_mean"]);
            ROS_ASSERT(class_control["linear_acceleration_std"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            double linear_acceleration_std = static_cast<double>(class_control["linear_acceleration_std"]);
            ROS_ASSERT(class_control["angular_velocity_std"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            double angular_velocity_std = static_cast<double>(class_control["angular_velocity_std"]);
            class_id_to_controller[class_id] = LinearAngularController(linear_acceleration_mean, angular_velocity_mean, linear_acceleration_std, angular_velocity_std);
            ROS_INFO_STREAM("Class controls for " << class_label << " are " << class_id_to_controller[class_id]);
        }
        return class_id_to_controller;
    }

    std::unordered_map<unsigned int, double> softmax(std::unordered_map<unsigned int, double> input_map, double beta = 5.0)
    {
        std::unordered_map<unsigned int, double> output_map;
        double sum = 0.0;

        // Calculate the exponential of each value in the input map
        for (const auto &kvp : input_map)
        {
            double exp_val = beta * exp(kvp.second);
            output_map[kvp.first] = exp_val;
            sum += exp_val;
        }

        // Normalize the values by dividing by the sum of exponentials
        for (auto &kvp : output_map)
        {
            kvp.second /= sum;
        }

        return output_map;
    }

    /**
     * Tracks the beliefs of all targets in a particular state space
     * Only one state space per Belief Manager because we apply observations within the same
     * state space to all trackers.
     */
    class ParticleFiltersBelief
    {
        //    protected:
    public:
        std::map<unsigned int, ParticleFilter> id_to_trackers;
        unsigned int NUM_PARTICLES;
        std::unordered_map<unsigned int, LinearAngularController> class_id_to_controller;
        double DISTANCE_STRICTNESS;
        double SOFTMAX_BETA;
        ros::NodeHandle pnh;

        // thresholds for accepting targets
        double min_z_accept_filtered_targets;
        double max_target_position_variance;
        double max_target_velocity_variance;

    public:
        ParticleFiltersBelief() = default;

        ParticleFiltersBelief(ros::NodeHandle &pnh)
            : pnh(pnh),
              NUM_PARTICLES(ros_utils::get_param<int>(pnh, "num_particles")),
              class_id_to_controller(parse_class_controls(ros_utils::get_param<XmlRpc::XmlRpcValue>(pnh, "class_controls"))),
              DISTANCE_STRICTNESS(ros_utils::get_param<double>(pnh, "distance_strictness")),
              SOFTMAX_BETA(ros_utils::get_param<double>(pnh, "softmax_beta")),
              min_z_accept_filtered_targets(ros_utils::get_param<double>(pnh, "min_z_accept_filtered_targets")),
              max_target_position_variance(ros_utils::get_param<double>(pnh, "max_target_position_variance")),
              max_target_velocity_variance(ros_utils::get_param<double>(pnh, "max_target_velocity_variance"))
        {
        }

        /**
         * Updates all trackers with time t
         * @param time_t
         */
        void propagate(double time_t)
        {
            for (auto &[id, tracker] : this->id_to_trackers)
            {
                tracker.propagate(time_t); // must use -> for polymorphism
            }
        }

        void hyper_propagate(double time_t, int num_steps = 1)
        {
            for (auto &[id, tracker] : this->id_to_trackers)
            {
                tracker.hyper_propagate(time_t, num_steps); // must use -> for polymorphism
            }
        }

        std::pair<bool, std::unordered_map<unsigned int, double>> get_association_probabilities(planner_map_interfaces::FilteredTarget &f_target)
        {
            bool is_likely_existing_target = false;
            std::unordered_map<unsigned int, double> prob_ids_given_target;

            bool is_global_id_provided = f_target.global_id != f_target.GLOBAL_ID_FOR_UNKNOWN;
            if (is_global_id_provided) // we're told the global id
            {
                ROS_WARN_STREAM("global id provided");
                is_likely_existing_target = this->id_to_trackers.count(f_target.global_id) > 0;
                // make a one-hot vector
                prob_ids_given_target.insert(std::make_pair(f_target.global_id, 1.0));
                for (auto &[id, particle_filter] : this->id_to_trackers)
                {
                    if (id != f_target.global_id)
                    {
                        prob_ids_given_target.insert(std::make_pair(id, 0.0));
                    }
                }
            }
            else // we're not told the global id, and so we must compute the probability of each tracker given the target
            {
                ROS_WARN_STREAM("global id not provided");
                double reassociation_probability = 0;
                // ROS_INFO_STREAM("Checking if incoming target local_id=" << f_target.local_id << " is a new target");
                // double previous_probability = 1.0;
                for (auto &[id, particle_filter] : this->id_to_trackers)
                {
                    if (particle_filter.get_class_id() != f_target.class_id) // only targets that match class_id
                    {
                        continue;
                    }
                    // conditional probability of tracker given context of others, hope this helps stop clustering of PFs
                    double p_tracker_matches_given_target = particle_filter.p_tracker_matches_given_target(f_target);
                    // double p_tracker_matches_given_target_and_not_previous_tracker = p_tracker_matches_given_target; // * (1.0 - previous_probability);
                    prob_ids_given_target.insert(std::make_pair(id, p_tracker_matches_given_target));
                    reassociation_probability += p_tracker_matches_given_target;
                    if (p_tracker_matches_given_target > 0.5)
                    {
                        is_likely_existing_target = true;
                    }
                    // previous_probability = p_tracker_matches_given_target;
                    // ROS_DEBUG_STREAM("Probabilty matches to tracker" << id << " is " << p_tracker_matches_given_target);
                }
                prob_ids_given_target = softmax(prob_ids_given_target, this->SOFTMAX_BETA);
            }
            return std::make_pair(is_likely_existing_target, prob_ids_given_target);
        }

        std::pair<bool, std::unordered_map<unsigned int, double>> get_ideal_association_probabilities(planner_map_interfaces::FilteredTarget &f_target)
        {

            bool is_likely_existing_target = true;
            std::unordered_map<unsigned int, double> prob_ids_given_target;
            prob_ids_given_target.insert(std::make_pair(f_target.local_id, 1.0));
            for (auto &[id, particle_filter] : this->id_to_trackers)
            {
                if (id != f_target.local_id)
                {
                    prob_ids_given_target.insert(std::make_pair(id, 0.0));
                }
            }
            return std::make_pair(is_likely_existing_target, prob_ids_given_target);
        }

        void apply_observation(Observation &observation)
        {
            if (observation.get_vantage_point().get_z() < this->min_z_accept_filtered_targets)
            {
                ROS_INFO_STREAM("Drone is too low z=" << observation.get_vantage_point().get_z() << " < min_z_accept_filtered_targets " << min_z_accept_filtered_targets << ", skipping observation");
                return;
            }
            // POSITIVE OBSERVATION
            for (auto &f_target : observation.get_filtered_targets().targets)
            {
                // TODO: global_id is having some issue, it doesn't get updated though the global_id is set in the plan request

                // auto [is_likely_existing_target, prob_ids_given_target] = this->get_association_probabilities(f_target);
                auto [is_likely_existing_target, prob_ids_given_target] = this->get_ideal_association_probabilities(f_target);

                // auto [is_likely_existing_target, prob_ids_given_target] = this->get_association_probabilities(f_target);
                // auto [is_likely_existing_target, prob_ids_given_target] = this->get_ideal_association_probabilities(f_target);

                double total_probability = std::accumulate(prob_ids_given_target.begin(), prob_ids_given_target.end(), 0.0, [](double sum, const std::pair<unsigned int, double> &p)
                                                           { return sum + p.second; });
                // note, we don't renormalize these weights because if the observation is reallyyy far from all targets,
                // then we only want to update the particle filters by a small weight
                if (is_likely_existing_target)
                {
                    for (auto &[id, particle_filter] : this->id_to_trackers)
                    {
                        ROS_WARN_STREAM("total p is " << total_probability);
                        double normalized_weight = prob_ids_given_target.at(id) / total_probability; // since we're sure this is one of the existing targets, we can renormalize
                        // ROS_DEBUG_STREAM("Applying local_id=" << f_target.local_id << " positive observation to tracker with normalized association weight P(global id=" << id << ")=" << normalized_weight << "");

                        if (normalized_weight > 0)
                        {
                            // particle_filter.apply_positive_observation(observation, f_target, normalized_weight);
                            particle_filter.apply_ideal_positive_observation(observation, f_target, normalized_weight);
                        }

                        // a previous version didn't check the normalized weight, keep in case
                        // particle_filter.apply_positive_observation(observation, f_target, normalized_weight);
                        // particle_filter.apply_ideal_positive_observation(observation, f_target, normalized_weight);

                    }
                }
                else // add a new target
                {

                    f_target.global_id = this->get_global_id_for_new_target();
                    this->add_new_target_to_belief(f_target);
                }
            }
            // NEGATIVE OBSERVATION
            if (observation.get_filtered_targets().targets.size() == 0)
            {
                for (auto &[id, particle_filter] : this->id_to_trackers)
                {
                    particle_filter.apply_negative_observation(observation);
                }
            }
        }

        void add_new_target_to_belief(const planner_map_interfaces::FilteredTarget &filtered_target, bool force_add = false)
        {
            if (!force_add && !does_meet_keep_criterion(filtered_target))
            {
                ROS_INFO_STREAM("Target local_id=" << filtered_target.local_id << " did not meet criterion for adding new tracker");
                return;
            }
            if (filtered_target.global_id == 0)
            {
                throw std::runtime_error("Target Global ID cannot be 0");
            }
            ROS_INFO_STREAM("Target local_id=" << filtered_target.local_id << " likely to be new and meets add criterion. Adding new tracker with global_id=" << filtered_target.global_id);

            unsigned int target_id = filtered_target.global_id;
            // this->target_id_to_time_last_seen.insert(std::make_pair(target_id, ros::Time::now()));

            ROS_INFO_STREAM("Target " << target_id << " of class label " << filtered_target.class_label << " class_id=" << filtered_target.class_id);

            try
            {
                auto target_gaussian = make_target_gaussian_from_filtered_target(filtered_target);

                auto target_particle_filter = ParticleFilter(
                    filtered_target.global_id,
                    filtered_target.class_id,
                    NUM_PARTICLES,
                    filtered_target.priority,
                    target_gaussian,
                    class_id_to_controller[filtered_target.class_id],
                    DISTANCE_STRICTNESS);
                this->add_tracker(target_particle_filter);
            }
            catch (const std::runtime_error &e)
            {
                ROS_ERROR_STREAM("Error adding new target to belief: " << e.what());
                return;
            }
        }

        bool does_meet_keep_criterion(const planner_map_interfaces::FilteredTarget &target)
        {
            double x_var = target.covariance[0];
            double y_var = target.covariance[5];
            double x_dot_var = target.covariance[10];
            double y_dot_var = target.covariance[15];
            ROS_DEBUG_STREAM("Criterion check: Target local_id=" << target.local_id << " has x_var=" << x_var << " y_var=" << y_var << " x_dot_var=" << x_dot_var << " y_dot_var=" << y_dot_var << "");
            return (
                (x_var <= this->max_target_position_variance && y_var <= this->max_target_position_variance) && (x_dot_var <= this->max_target_velocity_variance && y_dot_var <= this->max_target_velocity_variance));
        }

        unsigned int get_global_id_for_new_target()
        {
            if (id_to_trackers.size() == 0)
            {
                return 1;
            }
            unsigned int new_id = id_to_trackers.rbegin()->first + 1;
            return new_id;
        }

        /**
         * returns the number of active trackers
         * @return
         */
        int get_num_trackers() const
        {
            // return trackers.size();
            return id_to_trackers.size();
        }

        /**
         * @brief Get the number of trackers that have more than 0 particles
         *
         * @return int
         */
        int get_num_active_trackers() const
        {
            int num_active_trackers = 0;
            for (auto &tracker : id_to_trackers)
            {
                if (tracker.second.get_num_particles() > 0)
                {
                    num_active_trackers++;
                }
            }
            return num_active_trackers;
        }

        std::vector<unsigned int> get_tracker_ids()
        {
            std::vector<unsigned int> ids;
            for (auto &tracker : id_to_trackers)
            {
                ids.push_back(tracker.first);
            }
            return ids;
        }

        /**
         * @brief Checks if an id is currently being tracked
         *
         * @param ship_id
         * @return true
         * @return false
         */
        bool has_target(unsigned int ship_id)
        {
            return (id_to_trackers.find(ship_id) != id_to_trackers.end());
        }

        /**
         * @brief Add a single tracker to the belief manager, and optionally a visualizer.
         * Overwrites existing tracker if it exists.
         *
         * @param tracker
         */
        void add_tracker(ParticleFilter tracker)
        {
            // printf("Adding tracker \n");
            // make a local copy of the id, since we're moving the tracker_ ptr
            unsigned int id = tracker.get_id();
            // this->id_to_trackers[id] = tracker;
            // this->id_to_trackers.erase(id);
            this->id_to_trackers.insert(std::make_pair(id, tracker));
        }

        void remove_tracker_id(unsigned int id)
        {
            this->id_to_trackers.erase(id);
        }

        void clear_trackers()
        {
            this->id_to_trackers.clear();
        }

        /**
         * @brief Get the tracker that corresponds to the given id
         *
         * @param id
         * @return Tracker&
         */
        const ParticleFilter &get_tracker(unsigned int id) const
        {
            if (this->id_to_trackers.find(id) == this->id_to_trackers.end())
            {
                printf("Tried to get out of non-existent target id %d\n", id);
                throw std::runtime_error("ERROR: Tracker with id not found in belief manager\n");
            }
            return this->id_to_trackers.at(id);
        }

        ParticleFilter &get_editable_tracker(unsigned int id)
        {
            if (this->id_to_trackers.find(id) == this->id_to_trackers.end())
            {
                printf("Tried to get out of non-existent target id %d\n", id);
                throw std::runtime_error("ERROR: Tracker with id not found in belief manager\n");
            }
            return this->id_to_trackers.at(id);
        }

        const std::map<unsigned int, ParticleFilter> &get_id_to_trackers() const
        {
            return id_to_trackers;
        }
    };

    /* SERIALIZATION AND DESERIALIZATION TO/FROM ROS */

    /** COMMENT OUT THIS ONE, PASS BY REFERENCE IS DANGEROUS BECAUSE WE MAY DELETE TRACKERS FROM THE BELIEF_MANAGER. IF IT IS DELETED, WE GET SEGFAULTS
     * @brief Takes a the ParticleFitlers.msg ROS message and converts it to a belief manager
     *
     * @param msg
     * @return ParticleFiltersBelief
     */
    ParticleFiltersBelief belief_manager_from_ros_msg(const ipp_belief::ParticleFiltersBelief::ConstPtr &msg)
    {
        ParticleFiltersBelief belief_manager;
        msg->particle_filters;
        for (auto &pf_msg : msg->particle_filters)
        {
            ParticleFilter pf_tracker(pf_msg);
            belief_manager.add_tracker(pf_tracker);
        }
        return belief_manager;
    }
    /**
     * @brief Takes a the ParticleFitlers.msg ROS message and converts it to a belief manager. Pass by value version
     *
     * @param msg
     * @return ParticleFiltersBelief
     */
    ParticleFiltersBelief belief_manager_from_ros_msg(ipp_belief::ParticleFiltersBelief &msg)
    {
        ParticleFiltersBelief belief_manager;
        msg.particle_filters;
        for (auto &pf_msg : msg.particle_filters)
        {
            ParticleFilter pf_tracker(pf_msg);
            belief_manager.add_tracker(pf_tracker);
        }
        return belief_manager;
    }

    /**
     * @brief  Takes a belief manager and converts it to a ParticleFilters.msg ROS message
     *
     * @param belief_manager
     * @return ipp_belief::ParticleFilters
     */
    ipp_belief::ParticleFiltersBelief belief_manager_to_ros_msg(ParticleFiltersBelief &belief_manager)
    {
        ipp_belief::ParticleFiltersBelief msg;
        msg.header.frame_id = "local_enu";
        msg.header.stamp = ros::Time();

        for (auto &[id, tracker] : belief_manager.get_id_to_trackers())
        {
            msg.particle_filters.push_back(tracker.to_ros_msg());
        }
        return msg;
    }

} // namespace tracking
