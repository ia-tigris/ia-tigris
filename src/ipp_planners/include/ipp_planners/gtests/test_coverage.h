#include "ipp_planners/CoveragePlanner.h"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>  

void result_to_file(std::vector<std::vector<double>> &path, std::string path_name)
{
    std::ofstream myfile;
    myfile.open(path_name);
    for (int i = 0; i < path.size(); i++)
    {
        myfile << path[i][0] << ", " << path[i][1] << ", " << path[i][2] << ", " << path[i][3] << std::endl;
    }
    myfile.close();
}



class CoverageTestFixture: public ::testing::Test { 
protected:
    ipp::RectangleCoverage* coverage_planner;

public: 
   CoverageTestFixture( ) { 
        // initialization code here
        std::vector<double> sensor_offset = {0, 0, 110};
        double sensor_width = 10;
        double turn_radius = 5;
        coverage_planner = new ipp::RectangleCoverage(sensor_offset, sensor_width, turn_radius);
   }
   ~CoverageTestFixture( ) { 
        // initialization code here
        delete coverage_planner;
   } 
};

class CoverageTestFixtureOffset: public ::testing::Test { 
protected:
    ipp::RectangleCoverage* coverage_planner;

public: 
   CoverageTestFixtureOffset( ) { 
        // initialization code here
        std::vector<double> sensor_offset = {-20, 1, 110};
        double sensor_width = 10;
        double turn_radius = 5;
        coverage_planner = new ipp::RectangleCoverage(sensor_offset, sensor_width, turn_radius);
   }
   ~CoverageTestFixtureOffset( ) { 
        // initialization code here
        delete coverage_planner;
   } 
};

// No wind conditions
TEST_F(CoverageTestFixture, test1)
{
    double x0 = 0;
    double y0 = 0;
    double x1 = 30;
    double y1 = 20;

    std::vector<std::vector<double>> corners = {{x0, y0}, {x0, y1}, {x1, y1}, {x1, y0}};

    std::vector<std::vector<double>> path = coverage_planner->rect_coverage(corners);
    
    EXPECT_FLOAT_EQ(path[0][0], 0);
    EXPECT_FLOAT_EQ(path[0][1], 5);
    EXPECT_FLOAT_EQ(path[0][2], 110);
    EXPECT_FLOAT_EQ(path[0][3], 0);

    EXPECT_FLOAT_EQ(path[1][0], 30);
    EXPECT_FLOAT_EQ(path[1][1], 5);
    EXPECT_FLOAT_EQ(path[1][2], 110);
    EXPECT_FLOAT_EQ(path[1][3], 0);

    EXPECT_FLOAT_EQ(path[3][0], 0);
    EXPECT_FLOAT_EQ(path[3][1], 15);
    EXPECT_FLOAT_EQ(path[3][2], 110);
    EXPECT_FLOAT_EQ(path[3][3], 3.1415927);

    EXPECT_EQ(path.size(), 4);

    // std::cout << "Path: " << std::endl;
    // for (auto point : path)
    // {
    //     std::cout << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << std::endl;
    // }
}

TEST_F(CoverageTestFixture, test2)
{
    double x0 = 0;
    double y0 = 0;
    double x1 = 20;
    double y1 = 30;

    std::vector<std::vector<double>> corners = {{x0, y0}, {x0, y1}, {x1, y1}, {x1, y0}};

    std::vector<std::vector<double>> path = coverage_planner->rect_coverage(corners);

    EXPECT_FLOAT_EQ(path[0][0], 5);
    EXPECT_FLOAT_EQ(path[0][1], 0);
    EXPECT_FLOAT_EQ(path[0][2], 110);
    EXPECT_FLOAT_EQ(path[0][3], 1.5707964);

    EXPECT_FLOAT_EQ(path[1][0], 5);
    EXPECT_FLOAT_EQ(path[1][1], 30);
    EXPECT_FLOAT_EQ(path[1][2], 110);
    EXPECT_FLOAT_EQ(path[1][3], 1.5707964);

    EXPECT_FLOAT_EQ(path[3][0], 15);
    EXPECT_FLOAT_EQ(path[3][1], 0);
    EXPECT_FLOAT_EQ(path[3][2], 110);
    EXPECT_FLOAT_EQ(path[3][3], 4.71239);

    EXPECT_EQ(path.size(), 4);
    // result_to_file(path, "/home/moon/Downloads/results.csv");

    // std::cout << "Path: " << std::endl;
    // for (auto point : path)
    // {
    //     std::cout << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << std::endl;
    // }
}

TEST_F(CoverageTestFixture, rotate_M_PI_4)
{
    double theta = M_PI_4;
    double width = 30;
    double height = 20;

    std::vector<std::vector<double>> corners = {{0, 0}, //bottom left
                                                {0*cos(theta)-height*sin(theta), 0*cos(theta)+height*sin(theta)}, //top left
                                                {width*cos(theta)-height*sin(theta), width*cos(theta)+height*sin(theta)}, //top right
                                                {width*cos(theta)-0*sin(theta), width*cos(theta)+0*sin(theta)}}; //bottom right

    std::vector<std::vector<double>> path = coverage_planner->rect_coverage(corners);
    // std::cout << "Path: " << std::endl;
    // for (auto point : path)
    // {
    //     std::cout << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << std::endl;
    // }
}

TEST_F(CoverageTestFixture, rotate_M_PI_4_swap_corner_order)
{
    double theta = M_PI_4;
    double width = 30;
    double height = 20;

    std::vector<std::vector<double>> corners = {{0*cos(theta)-height*sin(theta), 0*cos(theta)+height*sin(theta)}, //top left
                                                {width*cos(theta)-height*sin(theta), width*cos(theta)+height*sin(theta)}, //top right
                                                {width*cos(theta)-0*sin(theta), width*cos(theta)+0*sin(theta)}, //bottom right
                                                {0, 0}}; //bottom left

    std::vector<std::vector<double>> path = coverage_planner->rect_coverage(corners);
    // std::cout << "Path: " << std::endl;
    // for (auto point : path)
    // {
    //     std::cout << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << std::endl;
    // }
}

TEST_F(CoverageTestFixtureOffset, test1)
{
    double x0 = 0;
    double y0 = 0;
    double x1 = 30;
    double y1 = 20;

    std::vector<std::vector<double>> corners = {{x0, y0}, {x0, y1}, {x1, y1}, {x1, y0}};

    std::vector<std::vector<double>> path = coverage_planner->rect_coverage(corners);
    // std::cout << "Path: " << std::endl;
    // for (auto point : path)
    // {
    //     std::cout << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << std::endl;
    // }
}

TEST_F(CoverageTestFixtureOffset, test2)
{
    double x0 = 0;
    double y0 = 0;
    double x1 = 20;
    double y1 = 30;

    std::vector<std::vector<double>> corners = {{x0, y0}, {x0, y1}, {x1, y1}, {x1, y0}};

    std::vector<std::vector<double>> path = coverage_planner->rect_coverage(corners);
    // std::cout << "Path: " << std::endl;
    // for (auto point : path)
    // {
    //     std::cout << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << std::endl;
    // }
}

TEST_F(CoverageTestFixtureOffset, rotate_M_PI_4)
{
    double theta = M_PI_4;
    double width = 30;
    double height = 20;

    std::vector<std::vector<double>> corners = {{0, 0}, //bottom left
                                                {0*cos(theta)-height*sin(theta), 0*cos(theta)+height*sin(theta)}, //top left
                                                {width*cos(theta)-height*sin(theta), width*cos(theta)+height*sin(theta)}, //top right
                                                {width*cos(theta)-0*sin(theta), width*cos(theta)+0*sin(theta)}}; //bottom right

    std::vector<std::vector<double>> path = coverage_planner->rect_coverage(corners);
    // std::cout << "Path: " << std::endl;
    // for (auto point : path)
    // {
    //     std::cout << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << std::endl;
    // }
}

TEST_F(CoverageTestFixtureOffset, rotate_M_PI_4_swap_corner_order)
{
    double theta = M_PI_4;
    double width = 30;
    double height = 20;

    std::vector<std::vector<double>> corners = {{0*cos(theta)-height*sin(theta), 0*cos(theta)+height*sin(theta)}, //top left
                                                {width*cos(theta)-height*sin(theta), width*cos(theta)+height*sin(theta)}, //top right
                                                {width*cos(theta)-0*sin(theta), width*cos(theta)+0*sin(theta)}, //bottom right
                                                {0, 0}}; //bottom left

    std::vector<std::vector<double>> path = coverage_planner->rect_coverage(corners);

    double epsilon = 0.001;
    EXPECT_NEAR(path[0][0], -25.4558, epsilon);
    EXPECT_NEAR(path[0][1], -2.82843, epsilon);
    EXPECT_NEAR(path[0][2], 110, epsilon);
    EXPECT_NEAR(path[0][3], 0.785398, epsilon);

    EXPECT_NEAR(path[1][0], -4.24264, epsilon);
    EXPECT_NEAR(path[1][1], 18.3848, epsilon);
    EXPECT_NEAR(path[1][2], 110, epsilon);
    EXPECT_NEAR(path[1][3], 0.785398, epsilon);

    EXPECT_NEAR(path[3][0], 11.313708, epsilon);
    EXPECT_NEAR(path[3][1], 16.970562, epsilon);
    EXPECT_NEAR(path[3][2], 110, epsilon);
    EXPECT_NEAR(path[3][3], 3.9269907, epsilon);

    EXPECT_EQ(path.size(), 4);

    // std::cout << "Path: " << std::endl;
    // for (auto point : path)
    // {
    //     std::cout << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << std::endl;
    // }
}