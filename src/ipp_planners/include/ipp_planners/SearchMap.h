#ifndef SEARCHMAP_H
#define SEARCHMAP_H

#include <tuple>
#include <vector>
#include <unordered_map>
#include <cmath>
#include "ipp_planners/TreeNode.h"
#include <eigen3/Eigen/Dense>
#include <float.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Polygon_2.h>
#include <geometry_msgs/Polygon.h>
#include "core_planning_state_space/state_spaces/xyzpsi_state_space.h"
#include "core_planning_state_space/state_space_utils/xyzpsi_state_space_utils.h"

#include "planner_map_interfaces/camera_projection.h"
#include "planner_map_interfaces/GridPrior.h"

#define TWO_PI 6.283185

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon_2;


/**
 * @brief  helper method to extract a vector of polygon points from a Polygon object
 * 
 * @param pose_array 
 * @return std::vector<std::vector<double>> 
 */
inline std::vector<std::vector<double>> extract_vector_polygon_bounds_from_polygon_msg(const geometry_msgs::Polygon &points_array)
{
    std::vector<std::vector<double>> polygon_bounds;
    for (int i = 0; i < points_array.points.size(); ++i)
    {
        polygon_bounds.push_back({points_array.points[i].x, points_array.points[i].y});
    }
    return polygon_bounds;
}


class SearchMap
{
private:
    double confidence_threshold;
    bool camera_within_footprint;
    // map on which calculations are performed. TODO: there's no reason for this to be a member attribute. convert it to a function parameter

public:
    SensorParams sensor_params;

    int x_start;
    int x_end;
    int y_start;
    int y_end;
    int map_resolution;
    int num_rows;
    int num_cols;
    double Rs = 1.0;
    double Rf = 1.0;
    double search_bounds_confidence = 0.000001;
    double search_bounds_priority = 1.0;
    int search_bounds_sensor_model = 0;
    double time_decay_slope;
    double time_decay_floor;
    bool bounds_convex;

    std::vector<std::vector<double>> polygon_bounds;
    std::vector<Point> CGAL_polygon_bounds;

    std::vector<planner_map_interfaces::GridPrior> prior_list;  // list of priors

    // static map for planning on
    std::vector<std::vector<double>> map;
    std::vector<std::vector<double>> copy_map;

    // map of structs to keep track of various traits at each grid point
    std::vector<std::vector<double>> priority;
    std::vector<std::vector<int>> sensor_model_id;

    SearchMap();

    SearchMap(
        std::vector<std::vector<double>> polygon_bounds,
        int x_start, 
        int y_start, 
        int x_end, 
        int y_end,
        int map_resolution,
        double confidence_threshold,
        SensorParams sensor_params,
        std::vector<double> initial_map_values={},
        std::vector<double> initial_priority_map={},
        std::vector<int> initial_sensor_id_map={},
        double time_decay_slope=0, 
        double time_decay_floor=1);

    // this function sets up the map with given map_resolution and priors
    void reset_map(std::vector<double> initial_map_values,
                   std::vector<double> initial_priority_map, 
                   std::vector<int> initial_sensor_id_map);

    template <typename T>
    std::vector<std::vector<T>> unflatten_map(std::vector<T> flattened_map, int num_rows, int num_cols);

    void update_priors();

    double search_information_gain(TreeNode *new_state, bool include_edge, double speed); 

    double search_information_gain_with_hashmaps(TreeNode *new_state, bool include_edge, double speed);

    std::vector<int> find_bounding_box(std::vector<std::vector<double>>& projected_camera_bounds, bool want_fully_within=true);

    static double substitute_point_in_line(std::vector<double>& pt1, std::vector<double>& pt2, int r, int c);

    bool check_cell_inside_convex_poly(int row, int col, std::vector<std::vector<double>>& projected_camera_bounds, bool want_fully_within=true);

    static bool check_point_inside_convex_poly(int x, int y, std::vector<std::vector<double>> &projected_camera_bounds);

    bool check_cell_inside_poly(int row, int col, std::vector<std::vector<double>>& projected_camera_bounds, bool want_fully_within=true);

    static bool check_point_inside_poly(int x, 
                                        int y,
                                        std::vector<std::vector<double>> &projected_camera_bounds,
                                        bool bounds_convex,
                                        const std::vector<Point> &CGAL_polygon_bounds);

    bool check_cell_inside_search_bounds(int row, int col, bool want_fully_within=true);

    bool check_point_inside_search_bounds(int &x, int &y);
    
    std::vector<double> find_footprint_centroid(std::vector<std::vector<double>> &projected_camera_bounds);

    double calc_reward_and_belief(double &range, double old_belief, double &new_belief, int &sensor_model_id);

    double estimate_belief_and_reward(std::vector<std::vector<double>> &projected_camera_bounds, 
                                      std::vector<double> &current_node, 
                                      std::vector<int> &bbox);

    double estimate_belief_and_reward_with_hashmap(TreeNode *new_state,
                                                   std::vector<std::vector<double>> &projected_camera_bounds, 
                                                   std::vector<double> &current_node, 
                                                   std::vector<int> &bbox);

    double estimate_edge_belief_and_reward(std::vector<std::vector<double>> &edgeGeometry, std::vector<int> &bbox, double center_slope,
                                           std::vector<std::vector<double>> projected_camera_bounds, double height_change, double start_height, std::vector<double> edge_end_pose,
                                           std::vector<std::vector<double>> end_footprint, double &time_elapsed);

    double estimate_edge_belief_and_reward_with_hashmap(TreeNode *new_state, std::vector<std::vector<double>> &edgeGeometry, std::vector<int> &bbox, double center_slope,
                                           std::vector<std::vector<double>> projected_camera_bounds, double height_change, double start_height, std::vector<double> edge_end_pose,
                                           std::vector<std::vector<double>> end_footprint, double &time_elapsed);

    double get_map_belief_hashmap(TreeNode *node, int &map_key);
    
    std::vector<std::vector<double>> find_edge_geometry(std::vector<double> edge_start_pose, std::vector<double> edge_end_pose, double max_range);

    std::vector<std::vector<double>> gridprior_to_vec(planner_map_interfaces::GridPrior& grid_prior);

    double calc_entropy(double& probability);

    double calc_time_reward_decay(double& time_elapsed);

    void fill_cells_on_line(std::vector<double>& p1, std::vector<double>& p2, double confidence, double priority, double sensor_model_id);
};

#endif