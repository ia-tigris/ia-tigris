#ifndef ipp_belief_STATISTICS_H
#define ipp_belief_STATISTICS_H

#include <ros/ros.h>
#include <functional>

#include "planner_map_interfaces/ros_utils.h"
#include "belief_manager.h"
#include "visualize.h"
#include "trackers.h"

namespace tracking
{

    double transform_variance(double variance, double variance_cap)
    {
        // double scale = 5e-5;  // maybe works well for bump function?
        if (variance <= 0)
        {
            return 0;
        }
        else if (variance > variance_cap)
        {
            ROS_DEBUG_STREAM_ONCE("variance transform is capped at " << variance_cap << ", but variance is " << variance 
            << ".  This might be happening a lot! Considering changing the variance cap");
            return 1;
        }
        else
        {
            // return std::exp(-1 / (scale * variance)); // bump function has a softening at low values
            // return 2 * 1 / (1 + exp(-scale * variance)) - 1;
            return (1 / variance_cap) * variance;
        }
    }

    /**
     * @brief Transforms through the transform function, then computes the information delta in transformed space
     *
     * @param prior the information before
     * @param posterior the information after
     * @return double information delta, ranges between [-1, 1]
     */
    double transform_variance_info_gain(double pre_var, double post_var, double variance_cap)
    {
        if (pre_var < 0 || post_var < 0)
            throw std::runtime_error("variance must be positive");
        double transformed_pre_var = transform_variance(pre_var, variance_cap);
        double transformed_post_var = transform_variance(post_var, variance_cap);

        double information_gain = transformed_pre_var - transformed_post_var;
        // if (information_gain < -1 || information_gain > 1)
        //     throw std::runtime_error("something fucked up, information gain must be between [-1, 1]");
        return information_gain;
    }

    // TODO: implement other notions of optimality. See https://en.wikipedia.org/wiki/Optimal_design#Minimizing_the_variance_of_estimators
    // A optimality seems straightforward, which is the trace, i.e. sum of the variances

    /**
     * @brief Calculates the average variance for all variables across all particles
     *
     * @param pf_tracker
     * @return double
     */
    double calculate_trace_variance(const ParticleFilter &pf_tracker)
    {
        auto &particles = pf_tracker.get_particles();
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

    /**
     * @brief Calculates the XY variance of the particles in the tracker
     *
     * @param pf_tracker
     * @return double
     */
    double calculate_xy_variance(const ParticleFilter &pf_tracker)
    {
        auto &particles = pf_tracker.get_particles();
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
        double pooled_var = (x_var + y_var) / 2.0;
        return pooled_var;
    }

    double calculate_avg_speed(const ParticleFilter &pf_tracker)
    {
        auto &particles = pf_tracker.get_particles();
        double speed_mean = 0.0;

        double n_samples = particles.size();
        // multiplying is cheaper than dividing
        double norm = 1.0 / n_samples;
        for (auto &particle : particles)
        {
            speed_mean += particle.get_speed() * norm;
        }

        return speed_mean;
    }

    /**
     * Get the total variance over a belief distribution
     * @param belief
     * @return
     */
    double calculate_belief_variance(const ParticleFiltersBelief &belief)
    {
        double total_var = 0;
        for (auto &id_and_tracker : belief.get_id_to_trackers())
        {
            auto &tracker = id_and_tracker.second;
            double var = calculate_trace_variance(tracker);
            total_var += var;
        }
        return total_var;
    }

    /**
     * @brief return the current XY variance for every tracker in the belief
     *
     * @param belief
     * @return std::vector<double>
     */
    std::vector<double> calculate_variance_per_tracker(const ParticleFiltersBelief &belief)
    {
        std::vector<double> variances;
        for (auto &id_and_tracker : belief.get_id_to_trackers())
        {
            auto &tracker = id_and_tracker.second;
            double var = calculate_trace_variance(tracker);
            variances.push_back(var);
        }
        return variances;
    }

    std::map<unsigned int, double> calculate_variance_per_tracker_map(const ParticleFiltersBelief &belief)
    {
        std::map<unsigned int, double> variances;
        for (auto &id_and_tracker : belief.get_id_to_trackers())
        {
            auto &tracker = id_and_tracker.second;
            double var = calculate_trace_variance(tracker);
            variances[id_and_tracker.first] = var;
        }
        return variances;
    }

    std::map<unsigned int, double> calculate_xy_variance_per_tracker_map(const ParticleFiltersBelief &belief)
    {
        std::map<unsigned int, double> variances;
        for (auto &id_and_tracker : belief.get_id_to_trackers())
        {
            auto &tracker = id_and_tracker.second;
            double var = calculate_xy_variance(tracker);
            variances[id_and_tracker.first] = var;
        }
        return variances;
    }

    std::map<unsigned int, double> calculate_avg_speed_per_tracker_map(const ParticleFiltersBelief &belief)
    {
        std::map<unsigned int, double> speeds;
        for (auto &id_and_tracker : belief.get_id_to_trackers())
        {
            auto &tracker = id_and_tracker.second;
            double speed = calculate_avg_speed(tracker);
            speeds[id_and_tracker.first] = speed;
        }
        return speeds;
    }

    std::vector<std::pair<double, double>> calculate_variance_per_tracker_w_priority(const ParticleFiltersBelief &belief)
    {
        std::vector<std::pair<double, double>> variances;
        for (auto &id_and_tracker : belief.get_id_to_trackers())
        {   
            auto &tracker = id_and_tracker.second;
            double var = calculate_trace_variance(tracker);
            variances.push_back(std::make_pair(var, tracker.get_priority()));
        }
        return variances;
    }

    std::vector<double> get_num_particles_per_tracker(const ParticleFiltersBelief &belief)
    {
        std::vector<double> num_particles;
        for (auto &id_and_tracker : belief.get_id_to_trackers())
        {
            auto &tracker = id_and_tracker.second;
            num_particles.push_back(tracker.get_num_particles());
        }
        return num_particles;
    }

    /**
     * @brief Counts the ratio of particles killed between observations. Weights particles killed by the variance.
     *
     * @param belief
     * @param observations
     * @param time_deltas
     * @param info_gain_function
     * @return std::pair<double, ParticleFiltersBelief>
     */
    std::pair<double, ParticleFiltersBelief> calculate_particle_info_gain(
        const ParticleFiltersBelief &belief,
        std::vector<Observation> &observations,
        std::vector<double> time_deltas,
        double variance_cap)
    {
        ParticleFiltersBelief belief_copy = belief;
        if (time_deltas.size() != observations.size())
        {
            throw std::runtime_error("number of time_deltas must match the number of observations");
        }

        std::vector<double> info_gain_per_tracker(belief_copy.get_id_to_trackers().size(), 0.0);

        // apply the observations iteratively
        for (int i = 0; i < observations.size(); i++)
        {
            std::vector<double> tracker_variances_pre = calculate_variance_per_tracker(belief_copy);
            std::vector<double> num_particles_pre = get_num_particles_per_tracker(belief_copy);

            auto &time_delta = time_deltas.at(i);
            auto &observation = observations.at(i);

            belief_copy.propagate(time_delta);
            belief_copy.apply_observation(observation);

            std::vector<double> num_particles_post = get_num_particles_per_tracker(belief_copy);

            std::vector<double> particle_difference_ratios;
            for (int i = 0; i < tracker_variances_pre.size(); i++)
            {
                double variance = tracker_variances_pre.at(i);
                double transformed_variance = transform_variance(variance, variance_cap);
                double num_particles_killed = num_particles_pre.at(i) - num_particles_post.at(i);
                assert(num_particles_killed >= 0);
                double particle_ratio = num_particles_pre.at(i) > 0 ? num_particles_killed / num_particles_pre.at(i) : 0;

                double info_gain = particle_ratio * transformed_variance;
                info_gain_per_tracker.at(i) += info_gain;
            }
        }

        double total_info_gain = std::accumulate(info_gain_per_tracker.begin(), info_gain_per_tracker.end(), 0.0);
        double average_info_gain = total_info_gain / info_gain_per_tracker.size();

        return std::make_pair(average_info_gain, belief_copy);
    }

    /**
     * @brief Calculate information gain over all targets using the given info_gain_function.
     *
     *
     * @param belief belief before update
     * @param observations list of discrete observations
     * @param time_deltas list of time deltas between observations
     * @param info_gain_function function to compute the information gain given prior and posterior information values
     * @return std::pair<double, ParticleFiltersBelief> positive info gain, and new resulting Belief
     */
    std::pair<double, ParticleFiltersBelief>
    calculate_variance_delta(
        const ParticleFiltersBelief &belief,
        std::vector<Observation> &observations,
        std::vector<double> time_deltas,
        std::function<double(double, double, double)> info_gain_function,
        double variance_cap
        )
    {
        ParticleFiltersBelief belief_copy = belief;
        if (time_deltas.size() != observations.size())
        {
            throw std::runtime_error("number of time_deltas must match the number of observations");
        }

        double total_info_gain = 0;

        // std::vector<double> tracker_variances_pre = calculate_variance_per_tracker(belief_copy);
        std::vector<std::pair<double, double>> tracker_variances_pre_w_priority = calculate_variance_per_tracker_w_priority(belief_copy);

        // apply the observations iteratively
        for (int i = 0; i < observations.size(); i++)
        {
            auto &time_delta = time_deltas.at(i);
            auto &observation = observations.at(i);

            belief_copy.propagate(time_delta);
            belief_copy.apply_observation(observation);
        }

        // ROS_WARN_STREAM("the ID size is " << belief.get_num_trackers());
        // std::vector<double> tracker_variances_post = calculate_variance_per_tracker(belief_copy);
        
        // calculate the variance difference per tracker
        // for (int j = 0; j < tracker_variances_pre.size(); j++)
        // {
        //     double var_pre = tracker_variances_pre.at(j);
        //     double var_post = tracker_variances_post.at(j);
        //     double information_gain = info_gain_function(var_pre, var_post);
        //     total_info_gain += information_gain;
        // }

        std::vector<std::pair<double, double>> tracker_variances_post_w_priority = calculate_variance_per_tracker_w_priority(belief_copy);
        /* 
        for (auto &var_and_pri : tracker_variances_post_w_priority)
        {
            ROS_WARN_STREAM("this target");
            ROS_WARN_STREAM("var is " << var_and_pri.first);
            ROS_WARN_STREAM("pri is " << var_and_pri.second);
        }
        */
        for (int j = 0; j < tracker_variances_pre_w_priority.size(); j++)
        {
            double var_pre = tracker_variances_pre_w_priority.at(j).first;
            double var_post = tracker_variances_post_w_priority.at(j).first;
            double var_priority = tracker_variances_post_w_priority.at(j).second;
            double information_gain = info_gain_function(var_pre, var_post, variance_cap);
            total_info_gain += information_gain * var_priority;
        }

        double average_info_gain = total_info_gain / tracker_variances_pre_w_priority.size();

        return std::make_pair(average_info_gain, belief_copy);
    }

    /**
     * @brief Calculate information gain over all targets using the given info_gain_function.
     *
     *
     * @param belief belief before update
     * @param observations list of discrete observations
     * @param time_deltas list of time deltas between observations
     * @param info_gain_function function to compute the information gain given prior and posterior information values
     * @return std::pair<double, ParticleFiltersBelief> positive info gain, and new resulting Belief
     */
    std::pair<double, ParticleFiltersBelief>
    calculate_variance_accumulated(
        const ParticleFiltersBelief &belief,
        std::vector<Observation> &observations,
        std::vector<double> time_deltas,
        double variance_cap
        )
    {
        ParticleFiltersBelief belief_copy = belief;
        if (time_deltas.size() != observations.size())
        {
            throw std::runtime_error("number of time_deltas must match the number of observations");
        }

        std::vector<double> tracker_variances(belief.get_num_trackers(), 0.0);

        // apply the observations iteratively
        for (int i = 0; i < observations.size(); i++)
        {
            auto &time_delta = time_deltas.at(i);
            auto &observation = observations.at(i);

            belief_copy.propagate(time_delta);
            belief_copy.apply_observation(observation);
            std::vector<double> tracker_variances_step = calculate_variance_per_tracker(belief_copy);
            for (int j = 0; j < tracker_variances.size(); j++)
            {
                tracker_variances.at(j) += time_delta * transform_variance(tracker_variances_step.at(j), variance_cap);
                // ROS_DEBUG_STREAM("obs " << i << " after time delta " << time_delta << ", tracker " << j + 1 << " accumulated variance: " << tracker_variances.at(j));
            }
        }
        // compute the averages
        auto tracker_ids = belief_copy.get_tracker_ids();
        for (int j = 0; j < tracker_variances.size(); j++)
        {
            auto &the_tracker = belief_copy.get_tracker(tracker_ids.at(j));
            // ROS_DEBUG_STREAM("tracker " << j + 1 << " accumulated variance: " << tracker_variances.at(j) << ". num particles : " << the_tracker.get_num_particles());
        }
        double total_variance_across_targets = std::accumulate(tracker_variances.begin(), tracker_variances.end(), 0.0);

        double avg_var_across_targets = total_variance_across_targets / tracker_variances.size();

        return std::make_pair(avg_var_across_targets, belief_copy);
    }

    /**
     * @brief Clips variance deltas to be positive. If the variance delta is negative, it is clipped to 0.
     * Only reduction in entropy is counted. Increase in entropy is ignored.
     *
     * @param pre_var the variance before
     * @param post_var the variance after
     * @return double information gain, strictly positive
     */
    double clip_to_positive_info_gain_only(double pre_var, double post_var)
    {
        double strictly_positive_info_gain = 0;
        double information_gain = pre_var - post_var;
        if (information_gain > 0)
            strictly_positive_info_gain += information_gain;
        if (information_gain < 0)
        {
            // ROS_DEBUG_STREAM("Information gain of tracker is " << information_gain << ", which is ne6gative. Clipping to 0");
            strictly_positive_info_gain += 0;
        }
        return strictly_positive_info_gain;
    }

} // namespace tracking

#endif // ipp_belief_STATISTICS_H