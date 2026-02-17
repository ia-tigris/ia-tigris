#ifndef INFORMATIVE_BELIEF_SEARCH_H
#define INFORMATIVE_BELIEF_SEARCH_H
#include <vector>
#include <cmath>
#include <planner_map_interfaces/TargetPrior.h>

#include "ipp_planners/InfoMap.h"
#include "ipp_planners/TreeNode.h"
#include "ipp_planners/SearchMap.h"
#include "ipp_belief/belief_manager.h"
#include <geometry_msgs/Polygon.h>

#include <ipp_planners/SearchMapSetup.h>
#include <ipp_planners/SearchMapUpdate.h>
#include "planner_map_interfaces/ros_utils.h"

namespace ipp
{
    /**
     * @brief Perform search objective. Samples based on entropy on a grid map.
     *
     */
    class InfoMapSearch : public InfoMap
    {
        /* ==== PUBLISHERS ==== */
    public:
        /* ==== MEMBER ATTRIBUTES ==== */
        std::discrete_distribution<> weighted_sampler;
        SearchMap local_map; // local copy of the latest search map

        bool setup_response;

        double flight_height;
        double viewpoint_goal;

        double time_decay_slope;
        double time_decay_floor;

        std::vector<std::vector<double>> search_bounds;
        std::vector<Point> CGAL_bounds;
        bool is_bounds_convex;
        bool use_hashmap = false;
        bool use_plan_horizon = false;

        InfoMapSearch(ros::NodeHandle &nh, ros::NodeHandle &pnh) : InfoMap(nh, pnh)
        {
            this->time_decay_slope = ros_utils::get_param<double>(nh, "realtime_search_map/time_decay_slope"); 
            this->time_decay_floor = ros_utils::get_param<double>(nh, "realtime_search_map/time_decay_floor");
            this->desired_speed = -1; // overwritten later with actual value from planning request
            this->flight_height = ros_utils::get_param<double>(nh, "realtime_search_map/flight_height");
            this->viewpoint_goal = ros_utils::get_param<double>(nh, "realtime_search_map/viewpoint_goal");
            this->use_hashmap = ros_utils::get_param<bool>(nh, "realtime_search_map/use_hashmap_for_search");
            this->use_plan_horizon = ros_utils::get_param<bool>(nh, "ipp_planners_node/use_plan_horizon", false);
        }

        /* =============================
         * ---- OVERRIDDEN METHODS ----
         * ============================= */

        void set_bounds(geometry_msgs::Polygon bounds) override{
            InfoMap::set_bounds(bounds); // super call

            this->search_bounds = extract_vector_polygon_bounds_from_polygon_msg(bounds);
            this->CGAL_bounds.clear();
            Point points[search_bounds.size()];
            for (int i = 0; i < search_bounds.size(); ++i)
            {
                this->CGAL_bounds.push_back(Point(search_bounds[i][0], search_bounds[i][1]));
            }
            Polygon_2 pgn(CGAL_bounds.begin(), CGAL_bounds.end());
            pgn.is_convex() ? this->is_bounds_convex = true : this->is_bounds_convex = false;
        }

        bool is_collision(XYZPsiStateSpace::StateType *state) override{
            double x = state->getX();
            double y = state->getY();
            if (this->strict_stay_in_bounds)
            {
                // if (this->is_bounds_convex)
                // {
                //     if(!SearchMap::check_point_inside_convex_poly((int) x, (int) y, this->search_bounds))
                //     {
                //         // ROS_WARN("Point is out of convex bounds");
                //         return true;
                //     }
                // }
                // else
                // {
                if (CGAL::bounded_side_2(this->CGAL_bounds.begin(),
                                            this->CGAL_bounds.end(),
                                            Point(x, y), K()) != CGAL::ON_BOUNDED_SIDE)
                {
                    // ROS_WARN("Point is out of concave bounds");
                    return true;
                }
                // }
            }
            // check if point in keep out zones
            for (int i = 0; i < this->keep_out_zones.size(); ++i)
            {
                double dx = x - this->keep_out_zones[i].center.first;
                double dy = y - this->keep_out_zones[i].center.second;
                double dist = sqrt(dx*dx + dy*dy);
                if (dist < this->keep_out_zones[i].radius)
                {
                    // ROS_WARN("Point is in keep out zone");
                    return true;
                }
            }
            return false;
        }

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
                ROS_INFO_STREAM("Keep out zone x: " << zone.x << " y: " << zone.y << " r: " << zone.radius);
            }

            if(msg.planner_params.header.stamp.sec != 0)
            {
                this->strict_stay_in_bounds = msg.planner_params.strict_stay_in_bounds;
                ROS_INFO_STREAM("Strict stay in bounds set to: " << this->strict_stay_in_bounds << " from plan request");
            }
            // Check if start state is within search bounds
            if (this->strict_stay_in_bounds == true){

                std::vector<Point> CGAL_bounds_;

                for (int i = 0; i < this->bounds.points.size(); ++i) {
                    CGAL_bounds_.push_back(Point(this->bounds.points[i].x, this->bounds.points[i].y));
                }

                if (CGAL::bounded_side_2(CGAL_bounds_.begin(),
                                         CGAL_bounds_.end(), 
                                         Point(msg.start_pose.position.x, msg.start_pose.position.y),
                                         K()) == CGAL::ON_UNBOUNDED_SIDE) {
                    ROS_WARN("Start state is not inside search bounds AND strict_stay_in_bounds is TRUE.\nSetting strict_stay_in_bounds to FALSE and continuing planning.\n");
                    this->strict_stay_in_bounds = false;
                }
            }
			return true;
        }

        /**
         * @brief Send target priors and map bounds to the search map
         * 
         * @param msg 
         * @return true 
         * @return false 
         */
        bool send_plan_request_params_to_belief(const planner_map_interfaces::PlanRequest &msg, double flight_height = 80) override
        {
            this->setup_response = false;
            ipp_planners::SearchMapSetup setup_srv;
            setup_srv.request.search_bounds = msg.search_bounds;
            setup_srv.request.target_priors = msg.target_priors;
            setup_srv.request.flight_height = flight_height;
            if (ros::service::call("realtime_search_map/search_map_setup", setup_srv))
            {
                ROS_INFO("map_setup_client call successful");
                this->setup_response = setup_srv.response.setup_response;
                return this->setup_response;
            }
            else
            {
                ROS_ERROR("Failed to call service map_setup_client");
                ros::Duration(1.0).sleep();
                return false;
            }
        }

        bool fetch_latest_belief(const std::vector<double> &pose_to_plan_from, const double &planning_budget) override
        {
            ipp_planners::SearchMapUpdate srv;
            if (this->setup_response && ros::service::call("realtime_search_map/search_map_update", srv))
            {
                // make a copy from the latest search map
                auto polygon_bounds = extract_vector_polygon_bounds_from_polygon_msg(srv.response.search_bounds);
                // ROS_INFO_STREAM("INSIDE BELIEF::"<<srv.response.x_start << "SENSOR PARAMS::"<<this->sensor_params.pitch);
                this->local_map = SearchMap(
                    polygon_bounds,
                    srv.response.x_start,
                    srv.response.y_start,
                    srv.response.x_end,
                    srv.response.y_end,
                    srv.response.map_resolution,
                    srv.response.confidence_threshold,
                    this->sensor_params,
                    srv.response.map_values,
                    srv.response.priority_map,
                    srv.response.sensor_model_id_map,
                    this->time_decay_slope, 
                    this->time_decay_floor
                );
                ROS_INFO_STREAM("Local planning search map updated");
                this->update_weighted_sampler(pose_to_plan_from, planning_budget);
            }
            else
            {
                ROS_ERROR("Failed to call service to update map");
                return false;
            }
            return true;
        }

        /**
         * @brief Sample an XY coordinate from the search coverage map
         *
         * @param tree_root
         * @return std::vector<double>
         */
        std::pair<double, double> sample_xy(const TreeNode &tree_root) override
        {
            if (!this->setup_response)
            {
                ROS_INFO_STREAM("SETUP RESPONSE 2 IS::" << this->setup_response);
                throw std::runtime_error("Search map has not been setup yet");
            }

            // Get cell from informed sampler
            auto sample_cell = weighted_sampler(gen);
            // printf("Trying to access local_map pointer\n");
            int row = sample_cell / local_map.num_cols;
            int col = sample_cell % local_map.num_cols;

            // Convert cell into x, y location
            double x = row * local_map.map_resolution + local_map.x_start + local_map.map_resolution / 2;
            double y = col * local_map.map_resolution + local_map.y_start + local_map.map_resolution / 2;
            return std::make_pair(x, y);
        }

        double calc_child_to_root_value(TreeNode &child_node, const bool is_edge_included = true) override
        {
            if (this->use_hashmap)
            {
                return local_map.search_information_gain_with_hashmaps(&child_node, is_edge_included, desired_speed);
            }

            return local_map.search_information_gain(&child_node, is_edge_included, desired_speed);
        }

        double calc_from_node_to_new_node_value(TreeNode &new_node, TreeNode &from_node, const bool is_edge_included = true)
        {
            return local_map.search_information_gain_with_hashmaps(&new_node, is_edge_included, desired_speed);
        }

        const SearchMap* get_search_map() const override
        {
            return &local_map;
        }

        std::unique_ptr<InfoMap> clone() 
        {
            return std::make_unique<InfoMapSearch>(*this);
        }

    private:
        /* =============================
         * ---- HELPER METHODS ----
         * ============================= */

        /*
        Updates the parameters needed for informed sampling
        */
         void update_weighted_sampler(const std::vector<double> &pose_to_plan_from, const double &planning_budget)
        {
            // Compute the reward for each cell in the map
            std::vector<double> weights;
            weights.reserve(local_map.num_rows * local_map.num_cols);

            double min_altitude = flight_height;
            double declination = local_map.sensor_params.pitch + viewpoint_goal * local_map.sensor_params.get_vfov() / 2; // Add because pitch is from forward direction.
            double range = min_altitude / std::sin(declination);

            // Check if the range is valid
            if (range >= local_map.sensor_params.highest_max_range){
                ROS_ERROR_STREAM("InfoMapSearch: Range is greater than the max range of the sensor. Range: " << range 
                    << " Max range: " << local_map.sensor_params.highest_max_range
                    << "\nThis could be due to the viewpoint_goal or max range being too low for given setup."
                    << "\n\n\n**************************\nThis will lead to poor results!!!!"
                    << " State sampler will not be able to sample valid states.\n**************************\n\n");
                ROS_WARN_STREAM("InfoMapSearch:Setting range to max range of sensor as a fallback.");
                range = local_map.sensor_params.highest_max_range;
            }

            // convert pose_to_plan_from to cell for comparisons
            int start_row = (int)((pose_to_plan_from[0] - (local_map.map_resolution/2) - local_map.x_start)/ local_map.map_resolution);
            int start_col = (int)((pose_to_plan_from[1] - (local_map.map_resolution/2) - local_map.y_start)/ local_map.map_resolution);

            // calculate cell radius we use for planning
            double buffer_from_pitch = min_altitude / tan(declination);; // Buffer because we can see beyond what underneith due to camera pitch.
            int cell_rad = floor(((planning_budget + buffer_from_pitch) - (local_map.map_resolution/2))/ local_map.map_resolution); //round down to nearest cell
            double cell_squared_rad = pow(cell_rad, 2.0);
            if (this->use_plan_horizon)
            {
                ROS_INFO_STREAM("Using start pose cell: (" << start_row << "," << start_col << ") with cell radius of " << cell_rad << " for sampling");
            }

            for (int i = 0; i < local_map.num_rows; i++)
            {
                for (int j = 0; j < local_map.num_cols; j++)
                {   
                    double new_belief = 0;
                    //distance we want to use for sampling
                    double euclid_squared_dist = pow(i-start_row, 2.0) + pow(j-start_col, 2.0);
                    if(this->use_plan_horizon && euclid_squared_dist > cell_squared_rad)
                    {
                        weights.push_back(new_belief);
                    }
                    else
                    {
                        // Scale weights by priority
                        weights.push_back(local_map.calc_reward_and_belief(range, local_map.map.at(i).at(j), new_belief, 
                            local_map.sensor_model_id.at(i).at(j)) * local_map.priority.at(i).at(j));
                    }
                }
            }
            // Make discrete distribution using reward as weight
            std::discrete_distribution<> disTemp(std::begin(weights), std::end(weights));
            weighted_sampler = disTemp;
        }
    };

}
#endif // INFORMATIVE_BELIEF_SEARCH_H
