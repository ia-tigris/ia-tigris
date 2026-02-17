//
// Created by andrew on 12/16/21.
//

#pragma once

#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <functional>
#include <random>

#include "control.h"
#include "observation.h"
#include "region.h"
#include "state.h"
#include <planner_map_interfaces/FilteredTarget.h>
#include <planner_map_interfaces/FilteredTargets.h>
#include <ipp_belief/ParticleFilterWithCtrlParams.h>
#include <ipp_belief/Particle.h>

namespace tracking
{
    /**
     * A generic Particle Filter Tracker.
     * @tparam StateSpace
     */
    class ParticleFilter
    {
    protected:
        unsigned int target_id;
        unsigned int class_id;
        LinearAngularController control_distribution;

        double duration_since_last_propagation = 0;
        const unsigned int NUM_PARTICLES;
        const double DISTANCE_STRICTNESS;

        double time_init;

        // last observed time, from time_init
        double last_observed_time;
        // Setup the random bits
        std::random_device rd;
        std::mt19937 gen;

        std::vector<TargetState> particles; // particles representing our belief

        double target_priority;

    public:
        ~ParticleFilter() = default;

        /**
         * @brief Main constructor for a new Particle Filter Tracker object
         *
         * @param id
         * @param steps_per_second
         * @param initial_state_distribution
         * @param control_distribution
         */
        ParticleFilter(unsigned int target_id,
                       unsigned int class_id,
                       unsigned int num_particles,
                       double priority,
                       TargetGaussian &initial_state_distribution,
                       LinearAngularController control_distribution, double distance_strictness)
            : target_id(target_id),
              class_id(class_id),
              NUM_PARTICLES(num_particles),
              particles(num_particles),
              target_priority(priority),
              control_distribution(control_distribution),
              DISTANCE_STRICTNESS(distance_strictness),
              time_init(ros::Time::now().toSec()),
              last_observed_time(0.0),
              gen(rd())
        {
            init_particles(initial_state_distribution);
        }

        // copy constructor
        ParticleFilter(const ParticleFilter &other)
            : target_id(other.target_id),
              class_id(other.class_id),
              NUM_PARTICLES(other.NUM_PARTICLES),
              particles(other.particles),
              control_distribution(other.control_distribution),
              duration_since_last_propagation(other.duration_since_last_propagation),
              DISTANCE_STRICTNESS(other.DISTANCE_STRICTNESS),
              target_priority(other.target_priority),
              time_init(other.time_init),
              last_observed_time(other.last_observed_time),
              gen(rd())
        {
        }

        /**
         * @brief Deserailize tracker from ROS message
         *
         * @param msg
         */
        ParticleFilter(const ipp_belief::ParticleFilterWithCtrlParams &msg)
            : target_id(msg.target_id),
              class_id(msg.class_id),
              NUM_PARTICLES(msg.num_particles),
              particles(msg.num_particles),
              target_priority(msg.target_priority),
              duration_since_last_propagation(msg.duration_since_last_propagation),
              DISTANCE_STRICTNESS(msg.distance_strictness),
              time_init(ros::Time::now().toSec()),
              last_observed_time(0.0),
              control_distribution(
                  msg.linear_acc_mean, msg.linear_acc_std, msg.angular_vel_mean, msg.angular_vel_std),
              gen(rd())
        {
            this->particles.clear();
            for (auto &p_msg : msg.particles)
            {
                TargetState particle(p_msg.id, p_msg.x, p_msg.y, p_msg.z, p_msg.h, p_msg.v, p_msg.w, p_msg.probability);
                this->particles.push_back(particle);
            }
        }

        /**
         * @brief Serialize to ROS message
         *
         * @return ipp_belief::ParticleFilterWithCtrlParams
         */
        ipp_belief::ParticleFilterWithCtrlParams to_ros_msg() const
        {
            ipp_belief::ParticleFilterWithCtrlParams msg;
            msg.target_id = target_id;
            msg.class_id = class_id;
            msg.num_particles = NUM_PARTICLES;
            msg.duration_since_last_propagation = duration_since_last_propagation;
            msg.target_priority = target_priority;
            msg.distance_strictness = DISTANCE_STRICTNESS;
            for (auto &p : this->particles)
            {
                ipp_belief::Particle p_msg;
                p_msg.id = p.get_id();
                p_msg.x = p.get_x();
                p_msg.y = p.get_y();
                p_msg.z = p.get_z();
                p_msg.h = p.get_heading();
                p_msg.v = p.get_speed();
                p_msg.w = p.get_angular_velocity();
                p_msg.probability = p.get_probability();
                msg.particles.push_back(p_msg);
            }
            // control params
            msg.linear_acc_mean = this->control_distribution.get_linear_acc_mean();
            msg.linear_acc_std = this->control_distribution.get_linear_acc_std();
            msg.angular_vel_mean = this->control_distribution.get_angular_vel_mean();
            msg.angular_vel_std = this->control_distribution.get_angular_vel_std();
            return msg;
        }

        /**
         * Initializes particles by sampling from the distribution of states
         *
         * @param state_distribution
         */
        void init_particles(TargetGaussian &state_distribution)
        {
            // ROS_DEBUG_STREAM("Initializing particles for target " << target_id);
            particles.clear();
            for (unsigned int i = 0; i < NUM_PARTICLES; ++i)
            {
                // printf("Sampling state index %d\n", i);
                // I ask for a state particle that I now own
                TargetState sampled_state = state_distribution.sample_state();
                // TODO: this was a hack, undo me at some point
                if (sampled_state.get_speed() < 0)
                {
                    sampled_state.set_speed(0.5 * -1 * sampled_state.get_speed());
                }
                assert(sampled_state.get_id() == this->target_id);
                sampled_state.set_probability(1.0 / NUM_PARTICLES);
                // ROS_DEBUG_STREAM("Sampled state: " << sampled_state);
                particles.push_back(sampled_state);
            }
        }

        /**
         *
         * @param delta_time
         * @param num_steps force  the number of steps. must be >= 1
         */
        void propagate(double delta_time, int num_steps = -1)
        {
            if (delta_time == 0)
            {
                return;
            }
            if (delta_time < 0)
            {
                throw std::runtime_error("Negative delta time: " + std::to_string(delta_time));
            }
            duration_since_last_propagation += delta_time;
            // Note AJ 2022.09.05. We disabled random sampling control, so hardcoding number of steps to 1 is valid.
            // Update AJ 2023.04.18 we renabled random sampling, haven't figured out yet how to discretize probabilistic control per time
            num_steps = 1;
            double dt_per_step = delta_time / num_steps;

            // propagate each particle for num_steps by duration dt_per_step.
            for (auto &particle : particles)
            {
                for (int i = 0; i < num_steps; i++)
                {
                    // TODO: make this RK4
                    control_distribution.apply_control(particle, dt_per_step);
                }
            }
        };

        void hyper_propagate(double delta_time, int num_steps = 1)
        {
            // hyper propagate
            if (delta_time == 0)
            {
                return;
            }
            if (delta_time < 0)
            {
                throw std::runtime_error("Negative delta time: " + std::to_string(delta_time));
            }
            duration_since_last_propagation += delta_time;
            
            double dt_per_step = delta_time / num_steps;

            // propagate each particle for num_steps by duration dt_per_step.
            for (auto &particle : particles)
            {
                for (int i = 0; i < num_steps; i++)
                {
                    control_distribution.apply_control(particle, dt_per_step);
                }
            }
        };

        /**
         * @brief Distance from particle to filtered_target taking into account the covariance of the target
         */
        double compute_mahalanobis_distance(TargetState &particle, const planner_map_interfaces::FilteredTarget &filtered_target)
        {
            Eigen::Matrix<double, 4, 4> target_cov(filtered_target.covariance.data());
            double particle_x = particle.get_x();
            double particle_y = particle.get_y();
            double particle_xdot = particle.get_speed() * std::cos(particle.get_heading());
            double particle_y_dot = particle.get_speed() * std::sin(particle.get_heading());

            // calculate mahalanobis distance
            Eigen::Matrix<double, 4, 1> diff;
            // AJ 2023.04.23 commented out velocity diff to see if it helps
            diff << filtered_target.x - particle_x, filtered_target.y - particle_y, (filtered_target.xdot - particle_xdot) * 0.01, (filtered_target.ydot - particle_y_dot) * 0.01;
            // ROS_DEBUG_STREAM("diff: " << diff);
            double mahalanobis_distance = diff.transpose() * target_cov.inverse() * diff;
            // ROS_DEBUG_STREAM("mahalanobis_distance: " << mahalanobis_distance);
            return mahalanobis_distance;
        }

        double compute_mahalanobis_probability(TargetState &particle, const planner_map_interfaces::FilteredTarget &filtered_target)
        {
            double mahalanobis_distance = compute_mahalanobis_distance(particle, filtered_target);
            double mahalanobis_probability = std::exp(-DISTANCE_STRICTNESS * mahalanobis_distance);
            return mahalanobis_probability;
        }

        /**
         * @brief Calculates the probability of a given target given the current state of the particles.
         * For each particle, computes the per-particle probability of the target given the particle's state.
         * This per-particle probability is an exponential of the negative mahalanobis distance between the particle's state and the target's state.
         * The total probability is the sum of the weighted per-particle probability.
         *
         * @param filtered_target
         * @return double
         */
        double p_tracker_matches_given_target(const planner_map_interfaces::FilteredTarget &filtered_target)
        {
            //
            double x = filtered_target.x;
            double y = filtered_target.y;
            double xdot = filtered_target.xdot;
            double ydot = filtered_target.ydot;

            Eigen::Matrix<double, 4, 4> target_cov(filtered_target.covariance.data());

            // TODO: these two for loops can be combined into one
            std::vector<double> per_particle_distance_probabilities;
            for (auto &particle : particles)
            {
                double probability = compute_mahalanobis_probability(particle, filtered_target);
                per_particle_distance_probabilities.push_back(probability);
            }

            // accumulate each particle's weight multiplied by its probability
            double total_probability = 0;
            for (int i = 0; i < particles.size(); i++)
            {
                total_probability += particles[i].get_probability() * per_particle_distance_probabilities[i];
            }
            return total_probability;
        }

        /**
         * @brief Applies a positive target observation to this tracker, optionally given the CONTEXT of other trackers in the belief.
         *
         *
         * @param observation the observation with drone vantage and sensor footprint to calculate the sensor model probability
         * @param f_target the specific target that was observed
         * @param p_is_this_tracker_given_f_target  P(this tracker matches the target | f_target) given other trackers
         */
        void apply_positive_observation(const Observation &obs, const planner_map_interfaces::FilteredTarget &f_target, double p_is_this_tracker_given_f_target = 1.0)
        {
            last_observed_time = ros::Time::now().toSec() - time_init;
            double p_obs_target_given_sensor_model = obs.calc_p_observation_given_particle(TargetState(f_target.class_id, f_target.x, f_target.y, 0, std::atan2(f_target.ydot, f_target.xdot), std::sqrt(f_target.xdot * f_target.xdot + f_target.ydot * f_target.ydot), 0, 1.0));
            ROS_DEBUG_STREAM("Apply positive observation to tracker " << this->target_id << " with p_is_this_tracker_given_f_target=" << p_is_this_tracker_given_f_target);
            for (int i = 0; i < particles.size(); i++)
            {
                auto &particle = particles[i];
                auto prev_weight = particle.get_probability();
                // ROS_DEBUG_STREAM("Particle idx=" << i << " prev_weight=" << prev_weight);
                double p_particle_given_f_target = compute_mahalanobis_probability(particle, f_target);
                // ROS_DEBUG_STREAM("p_particle_given_f_target: " << p_particle_given_f_target);
                double p_particle_if_not_f_target = 1.0;
                double p_is_not_this_tracker_given_f_target = 1.0 - p_is_this_tracker_given_f_target;
                double p_particle_weighted_by_p_tracker = (p_is_this_tracker_given_f_target * p_particle_given_f_target + p_is_not_this_tracker_given_f_target * p_particle_if_not_f_target);
                double new_probability = prev_weight * p_particle_weighted_by_p_tracker * p_obs_target_given_sensor_model;
                // ROS_DEBUG_STREAM("new_probability: " << new_probability);
                particle.set_probability(new_probability);
            }
            this->normalize_and_resample_weights([f_target](const TargetState& particle)
                                                 {
                    double f_target_speed = std::sqrt(f_target.xdot * f_target.xdot + f_target.ydot * f_target.ydot);
                    double f_target_heading = std::atan2(f_target.ydot, f_target.xdot);
                    Eigen::Matrix<double, 5, 5> noise_cov = Eigen::Matrix<double, 5, 5>::Identity();
                    noise_cov(0, 0) = 2 * std::abs(f_target.x - particle.get_x());      // x
                    noise_cov(1, 1) = 2 * std::abs(f_target.y - particle.get_y());      // y
                    noise_cov(2, 2) = 0.001 * std::abs(f_target_heading); // h this hardcoded number for heading can hopefully go away when we convert everthing to xdot ydot
                    noise_cov(3, 3) = std::abs(f_target_speed - particle.get_speed());     // linear speed
                    noise_cov(4, 4) = 0.00001;     // angular speed
                    return noise_cov; });
        }

        /**
         * @brief Applies a positive target observation to this tracker, optionally given the CONTEXT of other trackers in the belief.
         *
         *
         * @param observation the observation with drone vantage and sensor footprint to calculate the sensor model probability
         * @param f_target the specific target that was observed
         * @param p_is_this_tracker_given_f_target  P(this tracker matches the target | f_target) given other trackers
         */
        void apply_ideal_positive_observation(const Observation &obs, const planner_map_interfaces::FilteredTarget &f_target, double p_is_this_tracker_given_f_target = 1.0)
        {
            last_observed_time = ros::Time::now().toSec() - time_init;
            // ROS_WARN_STREAM("[ideal] detected target " << target_id << " at time " << last_observed_time);
            ROS_DEBUG_STREAM("Apply positive observation to tracker " << this->target_id << " with p_is_this_tracker_given_f_target=" << p_is_this_tracker_given_f_target);
            double f_target_speed = std::sqrt(f_target.xdot * f_target.xdot + f_target.ydot * f_target.ydot);
            double f_target_heading = std::atan2(f_target.ydot, f_target.xdot);

            Eigen::Matrix<double, 5, 5> noise_cov = Eigen::Matrix<double, 5, 5>::Identity();
            noise_cov(0, 0) = f_target.covariance[0]; // x
            noise_cov(1, 1) = f_target.covariance[5];      // y
            noise_cov(2, 2) = 0.001 * std::abs(f_target_heading); // h this hardcoded number for heading can hopefully go away when we convert everthing to xdot ydot
            noise_cov(3, 3) = 0.05 * (f_target.covariance[10] * f_target.covariance[10] + f_target.covariance[15] * f_target.covariance[15]);     // linear speed
            noise_cov(4, 4) = 0.00001;     // angular speed

            TargetState observed_state(target_id, f_target.x, f_target.y,
                                      0.0, f_target_heading, f_target_speed,
                                      0.0, 1.0);

            TargetGaussian re_init_gaussian(observed_state, noise_cov);

            init_particles(re_init_gaussian);
        }

        /**
         * @brief The observation does not contain the ground truth target. Therefore narrow
         * the belief by killing any particles within this observation footprint.
         * Any killed particles get resampled from the remaining particles.
         *
         * @param obs
         * @param sensor_model
         */
        void apply_negative_observation(const Observation &obs)
        {
            // update the weights of the particles
            bool must_resample = false;
            for (auto &particle : particles)
            {
                double p_obs = obs.calc_p_observation_given_particle(particle);
                if (obs.contains(particle))  // only resample if a particle is affected by the observation
                {
                    must_resample = true;
                    particle.set_probability(p_obs * particle.get_probability());
                }
            }
            if (must_resample)
            {
                ROS_DEBUG_STREAM("Apply negative observation to tracker " << this->target_id << ": " << obs.get_vantage_point().get_x() << ", " << obs.get_vantage_point().get_y() << ", " << obs.get_vantage_point().get_z());
                this->normalize_and_resample_weights([](const TargetState& particle)
                                                     {
                    Eigen::Matrix<double, 5, 5> noise_cov = Eigen::Matrix<double, 5, 5>::Identity();
                    noise_cov(0, 0) = 50;      // x
                    noise_cov(1, 1) = 50;      // y
                    noise_cov(2, 2) = 0.00001; // h
                    noise_cov(3, 3) = 0.1;     // linear speed
                    noise_cov(4, 4) = 0.00001;     // angular speed
                    return noise_cov; });
            }
        }

        // "low-variance" resampling method
        /**
         * @brief first normalize all the particle weights, then resample based on the normalized weights
         *
         * @param compute_p_obs_given_particle a function that computes the probability of the observation (you tell decide what the observation is) given a newly sampled particle's state
         */
        void normalize_and_resample_weights(std::function<Eigen::Matrix<double, 5, 5>(const TargetState)> calculate_noise_cov_for_resample)
        {
            // ROS_DEBUG_STREAM("normalize_and_resample_weights");
            this->normalize_weights();

            // todo: sample greater noise around particle corresponding to the difference between the particle and the mean

            // Get total weight of all particles
            double total_weight = 1.0;

            std::vector<TargetState> resampled_particles; // Create a vector to store resampled particles
            // Initialize random number generator
            std::random_device rd;
            std::mt19937 gen(rd());

            double stepSize = 1.0 / NUM_PARTICLES;
            std::uniform_real_distribution<> dis(0.0, stepSize);
            double r = dis(gen);
            double c = this->particles.at(0).get_probability();
            int index = 0;
            int last_index = -1;

            for (int m = 0; m < this->particles.size(); ++m)
            {
                double U = r + m * stepSize;
                while (U > c && index < this->particles.size() - 1)
                {
                    index++;
                    c += this->particles.at(index).get_probability();
                }

                auto sampled_particle = this->particles.at(index); // copies
                double prev_weight = sampled_particle.get_probability();
                // repeat particle, then sample around the particle with some noise
                if (last_index == index)
                {
                    // Add the selected particle to the resampled particles vector
                    TargetGaussian gaussian(sampled_particle, calculate_noise_cov_for_resample(sampled_particle));
                    sampled_particle = gaussian.sample_state();
                    // ROS_DEBUG_STREAM("new_particle sampled from noise around sampled_particle: " << sampled_particle);
                }
                sampled_particle.set_probability(prev_weight);
                resampled_particles.push_back(sampled_particle);

                last_index = index;
            }

            assert(resampled_particles.size() == NUM_PARTICLES);

            // Update particles with resampled particles
            this->particles.clear();
            this->particles.insert(this->particles.end(), resampled_particles.begin(), resampled_particles.end());
            // this->particles = resampled_particles;
            this->normalize_weights();
        }

        // normalize the weights so that they sum to 1
        void normalize_weights()
        {
            // ROS_DEBUG_STREAM("normalize_weights");
            double total_weight = 0;
            for (const auto &particle : particles)
            {
                total_weight += particle.get_probability();
            }
            if (total_weight != 1.0)
            {
                for (auto &particle : particles)
                {
                    double new_weight = particle.get_probability() / total_weight;
                    // ROS_DEBUG_STREAM("old weight=" << particle.get_probability() << ", total_weights="<< total_weight <<", new weight=" << new_weight << "");
                    particle.set_probability(new_weight);
                }
            }
        }

        double calculate_trace_variance()
        {
            auto &particles = this->get_particles();
            double x_mean = 0, y_mean = 0, z_mean = 0, h_mean = 0, v_mean = 0, w_mean = 0;

            double n_samples = particles.size();
            // multiplying is cheaper than dividing
            double norm = 1.0 / n_samples;
            for (auto &particle : particles)
            {
                x_mean += particle.get_x() * norm;
                y_mean += particle.get_y() * norm;
                z_mean += particle.get_z() * norm;
                h_mean += particle.get_heading() * norm;
                v_mean += particle.get_speed() * norm;
                w_mean += particle.get_angular_velocity() * norm;
            }

            double x_var = 0, y_var = 0, z_var = 0, h_var = 0, v_var = 0, w_var = 0;

            for (auto &particle : particles)
            {
                // formula difference^2 * 1/n
                x_var += (particle.get_x() - x_mean) * (particle.get_x() - x_mean) * norm;
                y_var += (particle.get_y() - y_mean) * (particle.get_y() - y_mean) * norm;
                z_var += (particle.get_z() - z_mean) * (particle.get_z() - z_mean) * norm;
                h_var += (particle.get_heading() - h_mean) * (particle.get_heading() - h_mean) * norm;
                v_var += (particle.get_speed() - v_mean) * (particle.get_speed() - v_mean) * norm;
                w_var += (particle.get_angular_velocity() - w_mean) * (particle.get_angular_velocity() - w_mean) * norm;
            }

            // this simple average works because the sample size of each variable is the same
            // double pooled_var = (x_var + y_var + z_var + h_var + v_var + w_var) / 6.0;

            // Note AJ 2022.09.05: disabled angular velocity becuase our intersection code is linear
            double pooled_var = (x_var + y_var + z_var + h_var + v_var) / 5.0;
            return pooled_var;
        }

        std::pair<double, double> calculate_xy_variance()
        {
            auto &particles = this->get_particles();
            double x_mean = 0.0;
            double y_mean = 0.0;

            double n_samples = particles.size();
            // multiplying is cheaper than dividing
            double norm = 1.0 / n_samples;
            for (auto &particle : particles)
            {
                x_mean += particle.get_x() * norm;
                y_mean += particle.get_y() * norm;
            }

            double x_var = 0.0;
            double y_var = 0.0;

            for (auto &particle : particles)
            {
                x_var += std::pow(particle.get_x() - x_mean, 2) * norm;
                y_var += std::pow(particle.get_y() - y_mean, 2) * norm;
            }

            // this simple average works because the sample sizes are the same
            return std::make_pair(x_var, y_var);
        }

        const std::vector<TargetState> &get_particles() const { return particles; }

        /**
         * @brief Get the mean of all the particles
         *
         * @return TargetState
         */
        TargetState get_mean_particle() const
        {

            double x = 0, y = 0, z = 0, h = 0, v = 0, w = 0;
            double hx = 0, hy = 0;
            for (auto &p : particles)
            {
                x += p.get_x();
                y += p.get_y();
                z += p.get_z();
                hx += std::cos(p.get_heading());
                hy += std::sin(p.get_heading());
                v += p.get_speed();
                w += p.get_angular_velocity();
            }
            x /= particles.size();
            y /= particles.size();
            z /= particles.size();
            h = std::fmod(std::atan2(hy, hx), 2 * M_PI);
            v /= particles.size();
            w /= particles.size();

            return TargetState(this->target_id, x, y, z, h, v, w, 1.0);
        }

        // samples a particle based on particle weights
        TargetState sample_particle_weighted() const
        {
            if (particles.size() == 0)
            {
                throw std::runtime_error("No particles to sample");
            }
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> particle_dis(0.0, 1.0);
            double particle_weight = particle_dis(gen);
            double cumulative_weight = 0.0;
            for (auto &particle : particles)
            {
                cumulative_weight += particle.get_probability();
                if (cumulative_weight >= particle_weight)
                {
                    return particle;
                }
            }
            // if we get here, we didn't find a particle
            throw std::runtime_error("No particle found");
        }

        // uniform sample particle
        TargetState sample_particle_uniform() const
        {
            if (particles.size() == 0)
            {
                throw std::runtime_error("No particles to sample");
            }
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> particle_dis(0, NUM_PARTICLES - 1);
            int particle_idx = particle_dis(gen);
            auto particle = get_particles().at(particle_idx); // copies
            return particle;
        }

        unsigned int get_id() const { return target_id; }

        unsigned int get_class_id() const { return class_id; }

        double get_priority() const { return target_priority; }

        double get_last_observed_time() const { return last_observed_time; }

        const LinearAngularController &get_control_distribution() const
        {
            return control_distribution;
        }
        /**
         * @brief Set a new control distribution
         *
         * @param control_distribution_
         */
        void set_control_distribution(
            LinearAngularController control_distribution_)
        {
            this->control_distribution = std::move(control_distribution_);
        }

        /**
         * Get the current number of particles. This may be less than max_particles if some
         * particles were killed.
         *
         * @return unsigned int
         */
        unsigned int get_num_particles() const { return particles.size(); }

        bool is_lost() const
        {
            return particles.size() == 0;
        }
    };

} // namespace tracking
