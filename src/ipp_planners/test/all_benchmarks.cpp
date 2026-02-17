#include <benchmark/benchmark.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <random>
#include "ipp_planners/SearchMap.h"
#include "benchmarks_SearchMap_hashmaps.cpp"
#include "benchmarks_SearchMap_info_gain.cpp"
#include "benchmarks_SearchMap_bounds.cpp"


int main(int argc, char** argv) 
{        
    ::benchmark::Initialize(&argc, argv);  
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) 
		return 1; 
	ros::init(argc, argv, "tester");
    ::benchmark::RunSpecifiedBenchmarks(); 
}                                                              