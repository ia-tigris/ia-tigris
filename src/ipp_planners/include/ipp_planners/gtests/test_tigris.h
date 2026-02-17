#include "ipp_planners/SearchMap.h"
#include "ipp_planners/Tigris.h"
#include "ipp_planners/GreedyTrackPlanner.h"
#include "ipp_planners/InfoMapTrack.h"
#include <gtest/gtest.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <iostream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;

void check_inside(Point pt, Point *pgn_begin, Point *pgn_end, K traits)
{
  // std::cout << "The point " << pt;
  switch(CGAL::bounded_side_2(pgn_begin, pgn_end,pt, traits)) {
    case CGAL::ON_BOUNDED_SIDE :
    //   std::cout << " is inside the polygon.\n";
      break;
    case CGAL::ON_BOUNDARY:
    //   std::cout << " is on the polygon boundary.\n";
      break;
    case CGAL::ON_UNBOUNDED_SIDE:
    //   std::cout << " is outside the polygon.\n";
      break;
  }
}

// Declare a test
TEST(TestSearchMap, checkPointInPolygon)
{
    SearchMap map;
	// std::chrono::steady_clock::time_point begin_total = std::chrono::steady_clock::now();
    
    // Check corner case
    double x = 1;
    double y = 1;
    std::vector<std::vector<double>> polygon_bounds = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    EXPECT_TRUE(map.check_point_inside_convex_poly((int) x, (int) y, polygon_bounds));
    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	// ROS_INFO_STREAM("Time for our poly check: " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count());


    // Check inside
    x = 1;
    y = 1;
    polygon_bounds = {{0, 0}, {0, 2}, {2, 2}, {2, 0}};

    // begin = std::chrono::steady_clock::now();
    EXPECT_TRUE(map.check_point_inside_convex_poly((int) x, (int) y, polygon_bounds));
    // end = std::chrono::steady_clock::now();
    // ROS_INFO_STREAM("Time for our poly check: " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count());

    // std::chrono::steady_clock::time_point end_total = std::chrono::steady_clock::now();
    // ROS_INFO_STREAM("Total time for our poly check: " << std::chrono::duration_cast<std::chrono::nanoseconds> (end_total - begin_total).count());

}

TEST(TestSearchMap, checkPointInPolygonCGAL)
{
    // std::chrono::steady_clock::time_point begin_total = std::chrono::steady_clock::now();
    // Check corner case
    double x = 1;
    double y = 1;
    std::vector<Point> polygon_bounds = { Point(0,0), Point(0,1), Point(1,1), Point(1,0)};

    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	EXPECT_TRUE(CGAL::bounded_side_2(polygon_bounds.begin(), polygon_bounds.end(), Point(x, y), K()) == CGAL::ON_BOUNDARY);
    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	// ROS_INFO_STREAM("Time for CGAL poly check: " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count());

    // Check inside
    x = 1;
    y = 1;
    polygon_bounds = { Point(0,0), Point(0,2), Point(2,2), Point(2,0)};

    // begin = std::chrono::steady_clock::now();
    EXPECT_TRUE(CGAL::bounded_side_2(polygon_bounds.begin(), polygon_bounds.end(), Point(x, y), K()) == CGAL::ON_BOUNDED_SIDE);
    // end = std::chrono::steady_clock::now();
    // ROS_INFO_STREAM("Time for CGAL poly check: " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count());

    // std::chrono::steady_clock::time_point end_total = std::chrono::steady_clock::now();
    // ROS_INFO_STREAM("Total time for CGAL poly check: " << std::chrono::duration_cast<std::chrono::nanoseconds> (end_total - begin_total).count());


	// Point points3[] = { Point(0,0), Point(4500,0), Point(4900,4900), Point(0,4900), Point(3000,2000)};
    // check_inside(Point(100, 2000), points3, points3+5, K());
    // EXPECT_TRUE(CGAL::bounded_side_2(points3, points3+5, Point(100, 2000), K()) != CGAL::ON_BOUNDED_SIDE);
    // check_inside(Point(4500, 100), points3, points3+5, K());
    // EXPECT_TRUE(CGAL::bounded_side_2(points3, points3+5, Point(4500, 100), K()) == CGAL::ON_BOUNDED_SIDE);
}

TEST(TestSearchMap, checkCellInPolygon)
{
    SearchMap map;
    map.map_resolution = 10;
    map.x_start = 0;
    map.y_start = 0;
    std::vector<std::vector<double>> polygon_bounds;
    int x = 0;
    int y = 0;

    // Check one corner in
    polygon_bounds = {{5, 5}, {20, 5}, {20, 20}, {5, 20}};
    EXPECT_TRUE(map.check_cell_inside_convex_poly(x, y, polygon_bounds, false));
    EXPECT_FALSE(map.check_cell_inside_convex_poly(x, y, polygon_bounds, true));

    // Check two corner in
    polygon_bounds = {{5, -5}, {20, -5}, {20, 20}, {5, 20}};
    EXPECT_TRUE(map.check_cell_inside_convex_poly(x, y, polygon_bounds, false));
    EXPECT_FALSE(map.check_cell_inside_convex_poly(x, y, polygon_bounds, true));

    // Check three corner in
    polygon_bounds = {{1, 0}, {20, 0}, {20, 20}, {0, 20}, {0, 1}};
    EXPECT_TRUE(map.check_cell_inside_convex_poly(x, y, polygon_bounds, false));
    EXPECT_FALSE(map.check_cell_inside_convex_poly(x, y, polygon_bounds, true));

    // Check four corner in
    polygon_bounds = {{-5, -5}, {20, -5}, {20, 20}, {-5, 20}};
    EXPECT_TRUE(map.check_cell_inside_convex_poly(x, y, polygon_bounds, false));
    EXPECT_TRUE(map.check_cell_inside_convex_poly(x, y, polygon_bounds, true));
}

class SearchMapFixture: public ::testing::Test { 
protected:
    SearchMap map;

public: 
   SearchMapFixture( ) { 
        // initialization code here
        map.map_resolution = 1;
        map.x_start = 0;
        map.y_start = 0;
        map.x_end = 10;
        map.y_end = 10;
        map.num_rows = static_cast<int>((map.x_end - map.x_start) / map.map_resolution);
        map.num_cols = static_cast<int>((map.y_end - map.y_start) / map.map_resolution);
   } 
};

TEST_F(SearchMapFixture, FindBoundingBoxFullyWithinTest)
{
    EXPECT_EQ(map.num_rows, 10);
    EXPECT_EQ(map.num_cols, 10);
    std::vector<std::vector<double>> projected_camera_bounds = {{-0.5, -0.5}, {1.9, -0.5}, {1.9, 3.5}, {-0.5, 3.5}};
    std::vector<int> bbox = map.find_bounding_box(projected_camera_bounds, true);
    EXPECT_EQ(bbox[0], 0);
    EXPECT_EQ(bbox[1], 0);
    EXPECT_EQ(bbox[2], 1);
    EXPECT_EQ(bbox[3], 3);
}

TEST_F(SearchMapFixture, FindBoundingBoxNotFullyWithinTest)
{
    std::vector<std::vector<double>> projected_camera_bounds = {{-0.5, -0.5}, {1.9, -0.5}, {1.9, 3.5}, {-0.5, 3.5}};
    std::vector<int> bbox = map.find_bounding_box(projected_camera_bounds, false);
    EXPECT_EQ(bbox[0], 0);
    EXPECT_EQ(bbox[1], 0);
    EXPECT_EQ(bbox[2], 2);
    EXPECT_EQ(bbox[3], 4);
}

TEST_F(SearchMapFixture, FindBoundingBoxFullyWithinEdgeCaseTest)
{
    std::vector<std::vector<double>> projected_camera_bounds = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    std::vector<int> bbox = map.find_bounding_box(projected_camera_bounds, true);
    EXPECT_EQ(bbox[0], 0);
    EXPECT_EQ(bbox[1], 0);
    EXPECT_EQ(bbox[2], 1);
    EXPECT_EQ(bbox[3], 1);
}

TEST_F(SearchMapFixture, FindBoundingBoxNotFullyWithinEdgeCaseTest)
{
    std::vector<std::vector<double>> projected_camera_bounds = {{-0.5, -0.5}, {10.5, -0.5}, {10.5, 10.5}, {-0.5, 10.5}};
    std::vector<int> bbox = map.find_bounding_box(projected_camera_bounds, false);
    EXPECT_EQ(bbox[0], 0);
    EXPECT_EQ(bbox[1], 0);
    EXPECT_EQ(bbox[2], 9);
    EXPECT_EQ(bbox[3], 9);
}

TEST_F(SearchMapFixture, FindBoundingBoxFullyWithinEdgeCaseTest2)
{
    std::vector<std::vector<double>> projected_camera_bounds = {{-0.5, -0.5}, {10.5, -0.5}, {10.5, 10.5}, {-0.5, 10.5}};
    std::vector<int> bbox = map.find_bounding_box(projected_camera_bounds, true);
    EXPECT_EQ(bbox[0], 0);
    EXPECT_EQ(bbox[1], 0);
    EXPECT_EQ(bbox[2], 9);
    EXPECT_EQ(bbox[3], 9);
}

TEST_F(SearchMapFixture, FindBoundingBoxNotFullyWithinEdgeCaseTest3)
{
    std::vector<std::vector<double>> projected_camera_bounds = {{-0.5, -0.5}, {9.5, -0.5}, {9.5, 9.5}, {-0.5, 9.5}};
    std::vector<int> bbox = map.find_bounding_box(projected_camera_bounds, false);
    EXPECT_EQ(bbox[0], 0);
    EXPECT_EQ(bbox[1], 0);
    EXPECT_EQ(bbox[2], 9);
    EXPECT_EQ(bbox[3], 9);
}

TEST_F(SearchMapFixture, FindBoundingBoxFullyWithinEdgeCaseTest3)
{
    std::vector<std::vector<double>> projected_camera_bounds = {{-0.5, -0.5}, {9.5, -0.5}, {9.5, 9.5}, {-0.5, 9.5}};
    std::vector<int> bbox = map.find_bounding_box(projected_camera_bounds, true);
    EXPECT_EQ(bbox[0], 0);
    EXPECT_EQ(bbox[1], 0);
    EXPECT_EQ(bbox[2], 9);
    EXPECT_EQ(bbox[3], 9);
}

TEST_F(SearchMapFixture, FindBoundingBoxNotFullyWithinEdgeCaseTest4)
{
    std::vector<std::vector<double>> projected_camera_bounds = {{-0.5, -0.5}, {8.5, -0.5}, {8.5, 8.5}, {-0.5, 8.5}};
    std::vector<int> bbox = map.find_bounding_box(projected_camera_bounds, false);
    EXPECT_EQ(bbox[0], 0);
    EXPECT_EQ(bbox[1], 0);
    EXPECT_EQ(bbox[2], 9);
    EXPECT_EQ(bbox[3], 9);
}

TEST_F(SearchMapFixture, FindBoundingBoxFullyWithinEdgeCaseTest4)
{
    std::vector<std::vector<double>> projected_camera_bounds = {{-0.5, -0.5}, {8.5, -0.5}, {8.5, 8.5}, {-0.5, 8.5}};
    std::vector<int> bbox = map.find_bounding_box(projected_camera_bounds, true);
    EXPECT_EQ(bbox[0], 0);
    EXPECT_EQ(bbox[1], 0);
    EXPECT_EQ(bbox[2], 8);
    EXPECT_EQ(bbox[3], 8);
}

TEST_F(SearchMapFixture, FindBoundingBoxFullyWithinSinglePointTest)
{
    std::vector<std::vector<double>> projected_camera_bounds = {{5.0, 5.0}};
    std::vector<int> bbox = map.find_bounding_box(projected_camera_bounds, true);
    EXPECT_EQ(bbox[0], 5);
    EXPECT_EQ(bbox[1], 5);
    EXPECT_EQ(bbox[2], 5);
    EXPECT_EQ(bbox[3], 5);
}

TEST(TestSearchMap, checkSearchInformationGain)
{
    ompl::base::SpaceInformationPtr si_xyzpsi = GetStandardXYZPsiSpacePtr();
    std::vector<std::vector<double>> polygon_bounds;
    polygon_bounds = {{-495, -495}, {-495, 49900}, {49900, 49900}, {45900, 25900}, {49900, -495}};
    int x_start = -500;
    int y_start = -500;
    int x_end = 50000;
    int y_end = 50000;
    int map_resolution = 10;
    double confidence_threshold = 0.5;
    std::vector<double> max_range = {610, 620, 700};
    double pitch = 1.57079632679; // Pitch down
    double desired_speed = 10;
    
    ros::NodeHandle nh;
    std::string path = ros::package::getPath("planner_map_interfaces");
    nh.setParam("/sensor/sensor_params_path", path + "/config/onr");

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


    // Test views out of bounds
    ob::ScopedState<XYZPsiStateSpace> start_n(si_xyzpsi);
    start_n->setXYZ({-1000, -1000, 10});
    start_n->setPsi(0);
    auto *node = new TreeNode(si_xyzpsi);
    si_xyzpsi->getStateSpace()->copyState(node->state, start_n.get());

    bool include_edge = true;
    double info_out = map.search_information_gain(node, include_edge, desired_speed);
    EXPECT_TRUE(info_out == 0);
    include_edge = false;
    info_out = map.search_information_gain(node, include_edge, desired_speed);
    EXPECT_TRUE(info_out == 0);


    // Test views in bounds
    double flight_height = 20;
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

    include_edge = true;
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    info_out = map.search_information_gain(node7, include_edge, desired_speed);
    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // ROS_INFO_STREAM("Time for check: " << std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count());
    // ROS_INFO_STREAM("Information gain: " << info_out);

    include_edge = false;
    // begin = std::chrono::steady_clock::now();
    double info_out_no_edge = map.search_information_gain(node7, include_edge, desired_speed);
    // end = std::chrono::steady_clock::now();
    // ROS_INFO_STREAM("Time for check: " << std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count());
    // ROS_INFO_STREAM("Information gain: " << info_out_no_edge);
    
    EXPECT_TRUE(info_out >= info_out_no_edge);
    double range = flight_height;
    int sensor_model_id = 0;
    double new_belief;
    double reward_one_cell = map.calc_reward_and_belief(range, confidence_threshold, new_belief, sensor_model_id);
    
    EXPECT_TRUE(abs(info_out - 24470*reward_one_cell) < 10);
    EXPECT_TRUE(info_out >= info_out_no_edge);
}

TEST(TestSearchMap, checkSearchInformationGainHashmap)
{
    double eps = 0.0001;
    
    ompl::base::SpaceInformationPtr si_xyzpsi = GetStandardXYZPsiSpacePtr();
    std::vector<std::vector<double>> polygon_bounds;
    polygon_bounds = {{-495, -495}, {-495, 49900}, {49900, 49900}, {45900, 25900}, {49900, -495}};
    int x_start = -500;
    int y_start = -500;
    int x_end = 50000;
    int y_end = 50000;
    int map_resolution = 10;
    double confidence_threshold = 0.5;
    std::vector<double> max_range = {610, 620, 700};
    double pitch = 1.57079632679; // Pitch down
    double desired_speed = 8;
    
    ros::NodeHandle nh;
    std::string path = ros::package::getPath("planner_map_interfaces");
    nh.setParam("/sensor/sensor_params_path", path + "/config/onr");

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


    // Test views out of bounds
    ob::ScopedState<XYZPsiStateSpace> start_n(si_xyzpsi);
    start_n->setXYZ({-1000, -1000, 10});
    start_n->setPsi(0);
    auto *node = new TreeNode(si_xyzpsi);
    si_xyzpsi->getStateSpace()->copyState(node->state, start_n.get());
    
    bool include_edge = true;
    double info_out = map.search_information_gain(node, include_edge, desired_speed);
    double info_out_hashmap = map.search_information_gain_with_hashmaps(node, include_edge, desired_speed);
    EXPECT_TRUE(info_out == 0);
    EXPECT_TRUE(info_out_hashmap == 0);
    include_edge = false;
    info_out = map.search_information_gain(node, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node, include_edge, desired_speed);
    EXPECT_TRUE(info_out == 0);
    EXPECT_TRUE(info_out_hashmap == 0);



    // Test views in bounds and include edge
    include_edge = true;
    double flight_height = 20;
    start_n->setXYZ({-5, -5, flight_height});
    start_n->setPsi(0);
    si_xyzpsi->getStateSpace()->copyState(node->state, start_n.get());
    info_out = map.search_information_gain(node, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node->information = info_out_hashmap;

    
    auto *node2 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({49005, -5, flight_height});
    start_n->setPsi(0);
    si_xyzpsi->getStateSpace()->copyState(node2->state, start_n.get());
    node2->straight_edge_start_pose = {-5, -5, flight_height, 0};
    node2->straight_edge_end_pose = {49005, -5, flight_height, 0};
    node2->parent = node;
    info_out = map.search_information_gain(node2, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node2, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node2->information = info_out_hashmap;

    auto *node3 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({49005, 49005, flight_height});
    start_n->setPsi(M_PI_2);
    si_xyzpsi->getStateSpace()->copyState(node3->state, start_n.get());
    node3->straight_edge_start_pose = {49005, -5, flight_height, M_PI_2};
    node3->straight_edge_end_pose = {49005, 49005, flight_height, M_PI_2};
    node3->parent = node2;
    info_out = map.search_information_gain(node3, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node3, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node3->information = info_out_hashmap;

    auto *node4 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({-5, 49005, flight_height});
    start_n->setPsi(M_PI);
    si_xyzpsi->getStateSpace()->copyState(node4->state, start_n.get());
    node4->straight_edge_start_pose = {49005, 49005, flight_height, M_PI};
    node4->straight_edge_end_pose = {-5, 49005, flight_height, M_PI};
    node4->parent = node3;
    info_out = map.search_information_gain(node4, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node4, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node4->information = info_out_hashmap;

    auto *node5 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({-5, 105, flight_height});
    start_n->setPsi(-M_PI_2);
    si_xyzpsi->getStateSpace()->copyState(node5->state, start_n.get());
    node5->straight_edge_start_pose = {-5, 49005, flight_height, -M_PI_2};
    node5->straight_edge_end_pose = {-5, 105, flight_height, -M_PI_2};
    node5->parent = node4;
    info_out = map.search_information_gain(node5, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node5, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node5->information = info_out_hashmap;

    auto *node6 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({40005, 105, flight_height});
    start_n->setPsi(0);
    si_xyzpsi->getStateSpace()->copyState(node6->state, start_n.get());
    node6->straight_edge_start_pose = {-5, 105, flight_height, 0};
    node6->straight_edge_end_pose = {40005, 105, flight_height, 0};
    node6->parent = node5;
    info_out = map.search_information_gain(node6, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node6, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node6->information = info_out_hashmap;

    auto *node7 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({40005, 48005, flight_height});
    start_n->setPsi(M_PI_2);
    si_xyzpsi->getStateSpace()->copyState(node7->state, start_n.get());
    node7->straight_edge_start_pose = {40005, 105, flight_height, M_PI_2};
    node7->straight_edge_end_pose = {40005, 48005, flight_height, M_PI_2};
    node7->parent = node6;
    info_out = map.search_information_gain(node7, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node7, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node7->information = info_out_hashmap;

    double info_out_total = info_out;
    double info_out_total_hashmap = info_out_hashmap;

    include_edge = false;
    double info_out_total_no_edge = map.search_information_gain(node7, include_edge, desired_speed);



    
    EXPECT_TRUE(info_out_total >= info_out_total_no_edge);
    double range = flight_height;
    int sensor_model_id = 0;
    double new_belief;
    double reward_one_cell = map.calc_reward_and_belief(range, confidence_threshold, new_belief, sensor_model_id);
    
    EXPECT_TRUE(abs(info_out - 24470*reward_one_cell) < 10);
    EXPECT_TRUE(info_out >= info_out_total_no_edge);
}

TEST(TestSearchMap, checkSearchInformationGainHashmapNoEdge)
{
    double eps = 0.0001;
    
    ompl::base::SpaceInformationPtr si_xyzpsi = GetStandardXYZPsiSpacePtr();
    std::vector<std::vector<double>> polygon_bounds;
    polygon_bounds = {{-495, -495}, {-495, 49900}, {49900, 49900}, {45900, 25900}, {49900, -495}};
    int x_start = -500;
    int y_start = -500;
    int x_end = 50000;
    int y_end = 50000;
    int map_resolution = 10;
    double confidence_threshold = 0.5;
    std::vector<double> max_range = {610, 620, 700};
    double pitch = 1.57079632679; // Pitch down
    double desired_speed = 10;
    
    ros::NodeHandle nh;
    std::string path = ros::package::getPath("planner_map_interfaces");
    nh.setParam("/sensor/sensor_params_path", path + "/config/onr");

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


    // Test views out of bounds
    ob::ScopedState<XYZPsiStateSpace> start_n(si_xyzpsi);
    start_n->setXYZ({-1000, -1000, 10});
    start_n->setPsi(0);
    auto *node = new TreeNode(si_xyzpsi);
    si_xyzpsi->getStateSpace()->copyState(node->state, start_n.get());
    
    bool include_edge = true;
    double info_out = map.search_information_gain(node, include_edge, desired_speed);
    double info_out_hashmap = map.search_information_gain_with_hashmaps(node, include_edge, desired_speed);
    EXPECT_TRUE(info_out == 0);
    EXPECT_TRUE(info_out_hashmap == 0);
    include_edge = false;
    info_out = map.search_information_gain(node, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node, include_edge, desired_speed);
    EXPECT_TRUE(info_out == 0);
    EXPECT_TRUE(info_out_hashmap == 0);



    // Test views in bounds and no edge
    include_edge = false;
    double flight_height = 20;
    start_n->setXYZ({-5, -5, flight_height});
    start_n->setPsi(0);
    si_xyzpsi->getStateSpace()->copyState(node->state, start_n.get());
    info_out = map.search_information_gain(node, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node->information = info_out_hashmap;

    
    auto *node2 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({49005, -5, flight_height});
    start_n->setPsi(0);
    si_xyzpsi->getStateSpace()->copyState(node2->state, start_n.get());
    node2->straight_edge_start_pose = {-5, -5, flight_height, 0};
    node2->straight_edge_end_pose = {49005, -5, flight_height, 0};
    node2->parent = node;
    info_out = map.search_information_gain(node2, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node2, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node2->information = info_out_hashmap;

    auto *node3 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({49005, 49005, flight_height});
    start_n->setPsi(M_PI_2);
    si_xyzpsi->getStateSpace()->copyState(node3->state, start_n.get());
    node3->straight_edge_start_pose = {49005, -5, flight_height, M_PI_2};
    node3->straight_edge_end_pose = {49005, 49005, flight_height, M_PI_2};
    node3->parent = node2;
    info_out = map.search_information_gain(node3, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node3, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node3->information = info_out_hashmap;

    auto *node4 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({-5, 49005, flight_height});
    start_n->setPsi(M_PI);
    si_xyzpsi->getStateSpace()->copyState(node4->state, start_n.get());
    node4->straight_edge_start_pose = {49005, 49005, flight_height, M_PI};
    node4->straight_edge_end_pose = {-5, 49005, flight_height, M_PI};
    node4->parent = node3;
    info_out = map.search_information_gain(node4, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node4, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node4->information = info_out_hashmap;

    auto *node5 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({-5, 105, flight_height});
    start_n->setPsi(-M_PI_2);
    si_xyzpsi->getStateSpace()->copyState(node5->state, start_n.get());
    node5->straight_edge_start_pose = {-5, 49005, flight_height, -M_PI_2};
    node5->straight_edge_end_pose = {-5, 105, flight_height, -M_PI_2};
    node5->parent = node4;
    info_out = map.search_information_gain(node5, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node5, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node5->information = info_out_hashmap;

    auto *node6 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({40005, 105, flight_height});
    start_n->setPsi(0);
    si_xyzpsi->getStateSpace()->copyState(node6->state, start_n.get());
    node6->straight_edge_start_pose = {-5, 105, flight_height, 0};
    node6->straight_edge_end_pose = {40005, 105, flight_height, 0};
    node6->parent = node5;
    info_out = map.search_information_gain(node6, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node6, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node6->information = info_out_hashmap;

    auto *node7 = new TreeNode(si_xyzpsi);
    start_n->setXYZ({40005, 48005, flight_height});
    start_n->setPsi(M_PI_2);
    si_xyzpsi->getStateSpace()->copyState(node7->state, start_n.get());
    node7->straight_edge_start_pose = {40005, 105, flight_height, M_PI_2};
    node7->straight_edge_end_pose = {40005, 48005, flight_height, M_PI_2};
    node7->parent = node6;
    info_out = map.search_information_gain(node7, include_edge, desired_speed);
    info_out_hashmap = map.search_information_gain_with_hashmaps(node7, include_edge, desired_speed);
    EXPECT_TRUE(abs(info_out - info_out_hashmap) < eps);
    node7->information = info_out_hashmap;

    double info_out_total = info_out;
    double info_out_total_hashmap = info_out_hashmap;
}

TEST(TestIntersect, test_greedy_intersection)
{
    double drone_x = 0;
    double drone_y = 0;
    double drone_speed = 1.000001;
    ipp::TargetCentroid new_centroid = {5, 5, 1, -PI/2, 0}; // x, y, speed, heading, var

    auto intersection = ipp::GreedyTrackPlanner::solve_soonest_intersection_drone_to_target(drone_x, drone_y, drone_speed, new_centroid);

    double drone_dist = sqrt(pow(intersection.first-drone_x, 2) + pow(intersection.second-drone_y, 2));
    double target_dist = sqrt(pow(intersection.first-new_centroid.x, 2) + pow(intersection.second-new_centroid.y, 2));
    double drone_time = drone_dist/drone_speed;
    double target_time = target_dist/new_centroid.speed;
    
    EXPECT_TRUE(abs(intersection.first - 5.0) < 0.01);
    EXPECT_TRUE(abs(intersection.second) < 0.01);
    EXPECT_TRUE(abs(drone_time-target_time) < 0.1);


    // Edge case where speeds are exactly the same, but should be able to find solution
    drone_speed = 1.0;

    intersection = ipp::GreedyTrackPlanner::solve_soonest_intersection_drone_to_target(drone_x, drone_y, drone_speed, new_centroid);

    drone_dist = sqrt(pow(intersection.first-drone_x, 2) + pow(intersection.second-drone_y, 2));
    target_dist = sqrt(pow(intersection.first-new_centroid.x, 2) + pow(intersection.second-new_centroid.y, 2));
    drone_time = drone_dist/drone_speed;
    target_time = target_dist/new_centroid.speed;
    
    EXPECT_TRUE(abs(intersection.first - 5.0) < 0.01);
    EXPECT_TRUE(abs(intersection.second) < 0.01);
    EXPECT_TRUE(abs(drone_time-target_time) < 0.1);


    // Check the times till intersection are the same for both vehicles
    drone_y = 1;
    drone_speed = 25;
    new_centroid = {200, 500, 20, 1.2, 0}; // x, y, speed, heading, var

    intersection = ipp::GreedyTrackPlanner::solve_soonest_intersection_drone_to_target(drone_x, drone_y, drone_speed, new_centroid);

    drone_dist = sqrt(pow(intersection.first-drone_x, 2) + pow(intersection.second-drone_y, 2));
    target_dist = sqrt(pow(intersection.first-new_centroid.x, 2) + pow(intersection.second-new_centroid.y, 2));
    drone_time = drone_dist/drone_speed;
    target_time = target_dist/new_centroid.speed;
    
    EXPECT_TRUE(abs(drone_time-target_time) < 0.1);
}

TEST(TestIntersect, test_tracking_intersection)
{
    double drone_x = 0;
    double drone_y = 0;
    double drone_speed = 1.000001;

    auto particle = tracking::TargetState(0, 5, 5, 110, -PI/2, 1, 0, 1.0);
    // ipp::TargetCentroid new_centroid = {5, 5, 1, -PI/2, 0}; // x, y, speed, heading, var

    auto intersection = ipp::InfoMapTrack::solve_soonest_intersection_drone_to_particle(drone_x, drone_y, drone_speed, particle);

    double drone_dist = sqrt(pow(intersection.first-drone_x, 2) + pow(intersection.second-drone_y, 2));
    double target_dist = sqrt(pow(intersection.first-particle.get_x(), 2) + pow(intersection.second-particle.get_y(), 2));
    double drone_time = drone_dist/drone_speed;
    double target_time = target_dist/particle.get_speed();
    
    EXPECT_TRUE(abs(intersection.first - 5.0) < 0.01);
    EXPECT_TRUE(abs(intersection.second) < 0.01);
    EXPECT_TRUE(abs(drone_time-target_time) < 0.1);


    // Edge case where speeds are exactly the same, but should be able to find solution
    drone_speed = 1.0;

    intersection = ipp::InfoMapTrack::solve_soonest_intersection_drone_to_particle(drone_x, drone_y, drone_speed, particle);

    drone_dist = sqrt(pow(intersection.first-drone_x, 2) + pow(intersection.second-drone_y, 2));
    target_dist = sqrt(pow(intersection.first-particle.get_x(), 2) + pow(intersection.second-particle.get_y(), 2));
    drone_time = drone_dist/drone_speed;
    target_time = target_dist/particle.get_speed();
    
    EXPECT_TRUE(abs(intersection.first - 5.0) < 0.01);
    EXPECT_TRUE(abs(intersection.second) < 0.01);
    EXPECT_TRUE(abs(drone_time-target_time) < 0.1);


    // Check the times till intersection are the same for both vehicles
    drone_y = 1;
    drone_speed = 25;

    particle = tracking::TargetState(0, 200, 500, 110, 1.2, 20, 0, 1.0);
    // ipp::TargetCentroid new_centroid = {200, 500, 20, 1.2, 0}; // x, y, speed, heading, var

    intersection = ipp::InfoMapTrack::solve_soonest_intersection_drone_to_particle(drone_x, drone_y, drone_speed, particle);

    drone_dist = sqrt(pow(intersection.first-drone_x, 2) + pow(intersection.second-drone_y, 2));
    target_dist = sqrt(pow(intersection.first-particle.get_x(), 2) + pow(intersection.second-particle.get_y(), 2));
    drone_time = drone_dist/drone_speed;
    target_time = target_dist/particle.get_speed();
    
    EXPECT_TRUE(abs(drone_time-target_time) < 0.1);
}

