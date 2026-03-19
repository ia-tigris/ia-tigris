#include <benchmark/benchmark.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <random>
#include "ipp_planners/SearchMap.h"

static void BM_SearchMap_Search_Information_Gain(benchmark::State& state) 
{
    ompl::base::SpaceInformationPtr si_xyzpsi = GetStandardXYZPsiSpacePtr();
    std::vector<std::vector<double>> polygon_bounds;
    polygon_bounds = {{-495, -495}, {-495, 49900}, {49900, 49900}, {45900, 25900}, {49900, -495}};
    int x_start = -500;
    int y_start = -500;
    int x_end = 50000;
    int y_end = 50000;
    int map_resolution = state.range(1);
    double confidence_threshold = 0.5;
    std::vector<double> max_range = {610, 620, 700};
    double pitch = 1.57079632679; // Pitch down
    double desired_speed = 10;
    
    ros::NodeHandle nh;
    std::string path = ros::package::getPath("planner_map_interfaces");
    nh.setParam("/sensor/sensor_params_path", path + "/config/fixed-wing");

    SensorParams sensor_params(15, 10, 10, pitch, max_range, 3);
    int num_rows = static_cast<int>((x_end - x_start) / map_resolution);
    int num_cols = static_cast<int>((y_end - y_start) / map_resolution);

    SearchMap map(polygon_bounds,
                    x_start, 
                    y_start, 
                    x_end, 
                    y_end,
                    map_resolution,
                    confidence_threshold,
                    sensor_params,
                    std::vector<double>(num_rows*num_cols, 0.5),
                    std::vector<double>(num_rows*num_cols, 1.0),
                    std::vector<int>(num_rows*num_cols, 0));


    double flight_height = 20;
    ob::ScopedState<XYZPsiStateSpace> start_n(si_xyzpsi);
    auto *node = new TreeNode(si_xyzpsi);
    start_n->setXYZ({-5, -5, flight_height});
    start_n->setPsi(0);
    si_xyzpsi->getStateSpace()->copyState(node->state, start_n.get());
    
    auto *node2 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({49005, -5, flight_height});
    start_n->setPsi(0);
    si_xyzpsi->getStateSpace()->copyState(node2->state, start_n.get());
    node2->straight_edge_start_pose = {-5, -5, flight_height, 0};
    node2->straight_edge_end_pose = {49005, -5, flight_height, 0};
    node2->parent = node;

    auto *node3 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({49005, 49005, flight_height});
    start_n->setPsi(M_PI_2);
    si_xyzpsi->getStateSpace()->copyState(node3->state, start_n.get());
    node3->straight_edge_start_pose = {49005, -5, flight_height, M_PI_2};
    node3->straight_edge_end_pose = {49005, 49005, flight_height, M_PI_2};
    node3->parent = node2;

    auto *node4 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({-5, 49005, flight_height});
    start_n->setPsi(M_PI);
    si_xyzpsi->getStateSpace()->copyState(node4->state, start_n.get());
    node4->straight_edge_start_pose = {49005, 49005, flight_height, M_PI};
    node4->straight_edge_end_pose = {-5, 49005, flight_height, M_PI};
    node4->parent = node3;

    auto *node5 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({-5, 105, flight_height});
    start_n->setPsi(-M_PI_2);
    si_xyzpsi->getStateSpace()->copyState(node5->state, start_n.get());
    node5->straight_edge_start_pose = {-5, 49005, flight_height, -M_PI_2};
    node5->straight_edge_end_pose = {-5, 105, flight_height, -M_PI_2};
    node5->parent = node4;

    auto *node6 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({40005, 105, flight_height});
    start_n->setPsi(0);
    si_xyzpsi->getStateSpace()->copyState(node6->state, start_n.get());
    node6->straight_edge_start_pose = {-5, 105, flight_height, 0};
    node6->straight_edge_end_pose = {40005, 105, flight_height, 0};
    node6->parent = node5;

    auto *node7 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({40005, 48005, flight_height});
    start_n->setPsi(M_PI_2);
    si_xyzpsi->getStateSpace()->copyState(node7->state, start_n.get());
    node7->straight_edge_start_pose = {40005, 105, flight_height, M_PI_2};
    node7->straight_edge_end_pose = {40005, 48005, flight_height, M_PI_2};
    node7->parent = node6;
    
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(map.search_information_gain(node7, state.range(0), desired_speed));
        benchmark::ClobberMemory();
    }     
}
BENCHMARK(BM_SearchMap_Search_Information_Gain)->Args({true, 10})->Args({false, 10})->Args({true, 8})->Args({false, 8});

static void BM_SearchMap_Estimtate_Belief_And_Reward(benchmark::State& state) 
{
    ompl::base::SpaceInformationPtr si_xyzpsi = GetStandardXYZPsiSpacePtr();
    std::vector<std::vector<double>> polygon_bounds;
    polygon_bounds = {{-495, -495}, {-495, 49900}, {49900, 49900}, {45900, 25900}, {49900, -495}};
    int x_start = -500;
    int y_start = -500;
    int x_end = 50000;
    int y_end = 50000;
    int map_resolution = state.range(0);
    double confidence_threshold = 0.5;
    std::vector<double> max_range = {610, 620, 700};
    double pitch = 1.57079632679; // Pitch down
    double desired_speed = 10;
    
    ros::NodeHandle nh;
    std::string path = ros::package::getPath("planner_map_interfaces");
    nh.setParam("/sensor/sensor_params_path", path + "/config/fixed-wing");

    SensorParams sensor_params(15, 10, 10, pitch, max_range, 3);
    int num_rows = static_cast<int>((x_end - x_start) / map_resolution);
    int num_cols = static_cast<int>((y_end - y_start) / map_resolution);

    SearchMap map(polygon_bounds,
                    x_start, 
                    y_start, 
                    x_end, 
                    y_end,
                    map_resolution,
                    confidence_threshold,
                    sensor_params,
                    std::vector<double>(num_rows*num_cols, 0.5),
                    std::vector<double>(num_rows*num_cols, 1.0),
                    std::vector<int>(num_rows*num_cols, 0));
    
    double yaw = 0.1;
    std::vector<double> current_node = {1230, 2230, 164, yaw};
    std::vector<std::vector<double>> q_rotated = rotated_camera_fov(sensor_params, /*roll*/ 0.0, /*sensor_params.pitch*/ sensor_params.pitch, /*yaw*/ yaw);
    std::vector<double> node_pos = {current_node[0], current_node[1], current_node[2]};
    std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pos, q_rotated, sensor_params.highest_max_range);
    std::vector<int> bbox = map.find_bounding_box(projected_camera_bounds);
    
    map.copy_map = map.map; 
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(map.estimate_belief_and_reward(projected_camera_bounds, current_node, bbox));
        benchmark::ClobberMemory();
    }     
}
BENCHMARK(BM_SearchMap_Estimtate_Belief_And_Reward)->Args({10})->Args({8});
                                                            