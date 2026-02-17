/**
 * @file observation.hpp
 * @author Andrew Jong (ajong@andrew.cmu.edu)
 * @brief represent robot observations
 * @version 0.1
 * @date 2022-02-13
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ipp_belief_OBSERVATION_H
#define ipp_belief_OBSERVATION_H

#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

#include <planner_map_interfaces/FilteredTargets.h>
#include "planner_map_interfaces/camera_projection.h"
#include "planner_map_interfaces/camera_projection.h"
#include "region.h"
#include "state.h"

namespace tracking
{
    /**
     * An observation describes a bounds containing a list of targets.
     *
     * An observation may have no targets, i.e. a negative observation. A negative
     * observation is still useful for updating the belief space with target absence.
     *
     */
    class Observation
    {
    protected:
        Polygon bounds;            // bounds in state space where the thing resides
        TargetState vantage_point; // vantage point of the observation
        // these are shared pointers because we don't necessarily own the distributions;

        planner_map_interfaces::FilteredTargets filtered_targets;

        std::set<unsigned int> target_ids;

        SensorParams sensor_params;

    public:
        Observation(Polygon bounds, TargetState vantage_point,
                    SensorParams sensor_params)
            : bounds(bounds),
              vantage_point(vantage_point),
              sensor_params(sensor_params)
        {
        }

        /**
         * Constructor.
         *
         * @param bounds the region that is visible by the observation
         * @param vantage_point a location from where the observation is being made.
         * used by the sensor model
         * targets in the observation
         * @param target_ids_to_distributions (optional, default: empty)
         */
        Observation(Polygon bounds, TargetState vantage_point,
                    SensorParams sensor_params,
                    planner_map_interfaces::FilteredTargets filtered_targets)
            : bounds(bounds),
              vantage_point(vantage_point),
              sensor_params(sensor_params),
              filtered_targets(filtered_targets)
        {
            for (auto &target : filtered_targets.targets)
            {
                target_ids.insert(target.global_id);
            }
        }

        // void add_filtered_target(planner_map_interfaces::FilteredTarget filtered_target){
        //     filtered_targets.targets.push_back(filtered_target);
        //     target_ids.insert(filtered_target.local_id);
        // }

        void set_bounds(Polygon bounds)
        {
            this->bounds = bounds;
        }

        /**
         * @brief Set the vantage point that this observation is being made from
         *
         * @param vantage_point
         */
        void set_vantage_point(TargetState vantage_point)
        {
            this->vantage_point = vantage_point;
        }

        /**
         * Get the region that the observation applies to. Returns a const reference
         *
         * @return Region
         */
        const Polygon &get_bounds() const { return bounds; }

        /**
         * @brief Checks if a state is within the observation bounds.  Short hand for
         * observation.get_bounds()->contains(state).
         *
         * @param state
         * @return true
         * @return false
         */
        bool contains(const TargetState &state) const
        {
            if (bounds.contains({state.get_x(), state.get_y()}))
                return true;
            return false;
        }

        bool has_target(unsigned int target_id) const
        {
            return target_ids.count(target_id);
        }

        /**
         * Get the vantage point of the observation
         *
         * @return TargetState
         */
        TargetState get_vantage_point() const { return vantage_point; }

        // getter
        planner_map_interfaces::FilteredTargets get_filtered_targets() { return filtered_targets; }

        /**
         * @brief Calculates the probability of observing this particle given the drone's vantage, 
         * our sensor footprint, and the tpr/fnr/fpr/tnr sensor models
         * 
         * @param state state to calculate the probability of observing
         * @return double the probability
         */
        double calc_p_observation_given_particle(const TargetState &state) const
        {
            int sensor_model_id = 0;
            if (this->contains(state))
            {
                // printf("Observation contains particle\n");
                double distance = this->get_vantage_point().euclidean_distance(state);

                // if the particle
                if (this->has_target(state.get_id()))
                {
                    // printf("applying true positive rate\n");
                    return sensor_params.tpr(distance, sensor_model_id);
                }
                else
                {
                    // printf("applying false negative rate\n");
                    return sensor_params.fnr(distance, sensor_model_id);
                }
            }
            // particle is not in the observation bounds
            else
            {
                // project the camera onto the ground
                double height = this->get_vantage_point().get_z();
                double downward_pitch = sensor_params.pitch;
                double distance = std::sin(downward_pitch) * height;
                // printf("Observation doesn't contain particle\n");
                // if the observation claims there's target, but the belief particle says we
                // shouldn't be able to see it, then this particle is not very probable
                if (this->has_target(state.get_id()))
                {

                    return sensor_params.fpr(distance, sensor_model_id);
                }
                // if the observation doesn't contain the target, and the belief particle
                // says we shouldn't be able to see it, then this particle is pretty
                // probable
                else
                {
                    // printf("applying true negative rate\n");
                    // return 1.0;
                    return sensor_params.tnr(distance, sensor_model_id);
                }
            }
        }

        void set_filtered_targets(planner_map_interfaces::FilteredTargets filtered_targets)
        {
            this->filtered_targets = filtered_targets;
            target_ids.clear();
            for (auto &target : filtered_targets.targets)
            {
                target_ids.insert(target.global_id);
            }
        }
    };

} // namespace tracking

#endif // ipp_belief_OBSERVATION_H
