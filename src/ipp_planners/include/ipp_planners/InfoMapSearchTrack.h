
#ifndef INFORMATIVE_BELIEF_SEARCH_TRACK_H
#define INFORMATIVE_BELIEF_SEARCH_TRACK_H
#include <random>

#include "ipp_planners/InfoMap.h"
#include "ipp_planners/InfoMapSearch.h"
#include "ipp_planners/InfoMapTrack.h"

namespace ipp
{
    /**
     * @brief Perform search objective. Samples based on entropy on a grid map.
     *
     */
    class InfoMapSearchTrack : public InfoMap
    {
        double prob_sample_track = 0.7;

        // multiobjective weights
        double search_info_weight = 1.0;
        double track_info_weight = 0.1;

        InfoMapSearch search;
        InfoMapTrack track;

    public:
        InfoMapSearchTrack(ros::NodeHandle &nh, ros::NodeHandle &pnh) : InfoMap(nh,pnh), search(nh,pnh), track(nh,pnh),
                                                  prob_sample_track(ros_utils::get_param<double>(pnh, "prob_sample_track")),
                                                  search_info_weight(ros_utils::get_param<double>(pnh, "search_reward_weight")),
                                                  track_info_weight(ros_utils::get_param<double>(pnh, "track_reward_weight"))
        {
            ROS_INFO("InfoMapSearchTrack initialized");
        }

        /* =============================
         * ---- OVERRIDEN METHODS ----
         * ============================= */
        bool save_plan_request_params(const planner_map_interfaces::PlanRequest &msg) override
        {
            if (msg.planner_params.use_gimbal)
                this->sensor_params = fetch_sensor_params_from_rosparam_server_for_gimbal_planner(this->nh);
            this->desired_speed = msg.desired_speed;
            this->set_bounds(msg.search_bounds);
            this->counter_detect_radius = msg.counter_detection_range; 
            this->keep_out_zones.clear();
            for (auto &zone : msg.keep_out_zones)
            {
                KeepOutZone new_koz;
                new_koz.center = std::make_pair(zone.x, zone.y);
                new_koz.radius = zone.radius;
                this->keep_out_zones.push_back(new_koz);
            }
            return this->search.save_plan_request_params(msg) && this->track.save_plan_request_params(msg);
        }

        bool send_plan_request_params_to_belief(const planner_map_interfaces::PlanRequest &msg, double flight_height = 80) override
        {
            if(msg.planner_params.header.stamp.sec != 0){
                this->prob_sample_track = msg.planner_params.prob_sample_track;
                this->search_info_weight = msg.planner_params.search_reward_weight;
                this->track_info_weight = msg.planner_params.track_reward_weight;

                ROS_INFO_STREAM("Prob sample track set to: " << this->prob_sample_track);
                ROS_INFO_STREAM("Search reward weight set to: " << this->search_info_weight);
                ROS_INFO_STREAM("Track reward weight set to: " << this->track_info_weight);
            }

            bool search_success = search.send_plan_request_params_to_belief(msg, flight_height);
            bool track_success = track.send_plan_request_params_to_belief(msg);
            return search_success && track_success;
        }

        /**
         * @brief Gives the info map the opportunity to make a service call to update the belief
         * with the new plan request
         *
         * @param msg
         */
        bool fetch_latest_belief(const std::vector<double> &pose_to_plan_from, const double &planning_budget) override
        {
            return search.fetch_latest_belief(pose_to_plan_from, planning_budget) 
                && track.fetch_latest_belief(pose_to_plan_from, planning_budget);
        }

        /**
         * @brief Sample an XY coordinate from the search coverage map
         *
         * @param tree_root
         * @return std::vector<double>
         */
        std::pair<double, double> sample_xy(const TreeNode &tree_root) override
        {
            bool choose_track =
                track.get_num_tracks() > 0 // only sample if there are tracks to sample from
                &&
                (0.01 * (rand() % 100)) < this->prob_sample_track; // choose track N% of the time

            // ROS_INFO_STREAM("Num trackers: " << track.get_num_tracks());
            if (choose_track)
            {
                // ROS_INFO("Sampling from track");
                return track.sample_xy(tree_root);
            }
            else
            {
                // ROS_INFO("Sampling from search");
                return search.sample_xy(tree_root);
            }
        }

        bool is_collision(XYZPsiStateSpace::StateType *state) override
        {
            return search.is_collision(state) || track.is_collision(state);
        }

        double calc_child_to_root_value(TreeNode &child_node, const bool is_edge_included = true) override
        {
            double search_info = this->search_info_weight<0.0001 ? 0 : 
                                 search.calc_child_to_root_value(child_node, is_edge_included);
            double track_info = track.get_num_tracks()==0 ? 0 : 
                                this->track_info_weight<0.0001 ? 0 : track.calc_child_to_root_value(child_node, is_edge_included);
            // ROS_INFO_STREAM("Search info gain:" << search_info << ", track info gain: " << track_info);
            double information = this->search_info_weight * search_info + this->track_info_weight * track_info;
            return information;
        }

        const SearchMap* get_search_map() const override
        {
            return search.get_search_map();
        }

        std::unique_ptr<InfoMap> clone()
        {
            return std::make_unique<InfoMapSearchTrack>(*this);
        }

        void set_desired_speed(double speed) override
        {
            this->desired_speed = speed;
            this->track.set_desired_speed(speed);
            this->search.set_desired_speed(speed);
        }

        /* =============================
         * ---- HELPER METHODS ----
         * ============================= */

        double get_max_edge_size()
        {
            return this->search.get_max_edge_size();
        }

    private:
    };

}
#endif // INFORMATIVE_BELIEF_SEARCH_TRACK_H
