#include <benchmark/benchmark.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <random>
#include "ipp_planners/SearchMap.h"

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

/*
Check in bounds benchmarking 
*/

static void BM_SearchMap_PointInPoly_Edge_Ours(benchmark::State& state) {
    SearchMap map;
    double x = 1;
    double y = 1;
    std::vector<std::vector<double>> polygon_bounds = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
    for (auto _ : state)
        map.check_point_inside_convex_poly((int) x, (int) y, polygon_bounds);
}
BENCHMARK(BM_SearchMap_PointInPoly_Edge_Ours);

static void BM_SearchMap_PointInPoly_Edge_CGAL(benchmark::State& state) {
    double x = 1;
    double y = 1;
    std::vector<Point> CGAL_bounds = { Point(0,0), Point(0,1), Point(1,1), Point(1,0)};
    for (auto _ : state)
        CGAL::bounded_side_2(CGAL_bounds.begin(),
                             CGAL_bounds.end(),
                             Point(x, y), K());
}
BENCHMARK(BM_SearchMap_PointInPoly_Edge_CGAL);

static void BM_SearchMap_PointInPoly_In_Ours(benchmark::State& state) {
    SearchMap map;
    double x = 1;
    double y = 1;
    std::vector<std::vector<double>> polygon_bounds = {{0, 0}, {0, 2}, {2, 2}, {2, 0}};
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(map.check_point_inside_convex_poly((int) x, (int) y, polygon_bounds));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_SearchMap_PointInPoly_In_Ours);

static void BM_SearchMap_PointInPoly_In_CGAL(benchmark::State& state) {
    double x = 1;
    double y = 1;
    std::vector<Point> CGAL_bounds = { Point(0,0), Point(0,2), Point(2,2), Point(2,0)};
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(CGAL::bounded_side_2(CGAL_bounds.begin(),
                                CGAL_bounds.end(),
                                Point(x, y), K()) == CGAL::ON_BOUNDED_SIDE);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_SearchMap_PointInPoly_In_CGAL);

static void BM_SearchMap_PointInPoly_Out_Ours(benchmark::State& state) {
    SearchMap map;
    double x = -1500;
    double y = 1500;
    std::vector<std::vector<double>> polygon_bounds = {{0, 0}, {0, 1000}, {1000, 1000}, {1000, 0}};
    for (auto _ : state)
        map.check_point_inside_convex_poly((int) x, (int) y, polygon_bounds);
}
BENCHMARK(BM_SearchMap_PointInPoly_Out_Ours);

static void BM_SearchMap_PointInPoly_Out_CGAL(benchmark::State& state) {
    double x = -1500;
    double y = 1500;
    std::vector<Point> CGAL_bounds = { Point(0,0), Point(0,1000), Point(1000,1000), Point(1000,0)};
    for (auto _ : state)
        CGAL::bounded_side_2(CGAL_bounds.begin(),
                             CGAL_bounds.end(),
                             Point(x, y), K());
}
BENCHMARK(BM_SearchMap_PointInPoly_Out_CGAL);

static void BM_SearchMap_PointInPoly_Rand_Ours_NoOpt(benchmark::State& state) {
    SearchMap map;
    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-200, 1200);

    std::vector<std::vector<double>> polygon_bounds = {{0, 0}, {0, 1000}, {1000, 1000}, {1000, 0}};
    for (auto _ : state)
        benchmark::DoNotOptimize(map.check_point_inside_convex_poly((int) disRange(gen), (int) disRange(gen), polygon_bounds));
}
BENCHMARK(BM_SearchMap_PointInPoly_Rand_Ours_NoOpt);

static void BM_SearchMap_PointInPoly_Rand_CGAL_NoOpt(benchmark::State& state) {
    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-200, 1200);

    std::vector<Point> CGAL_bounds = { Point(0,0), Point(0,1000), Point(1000,1000), Point(1000,0)};
    for (auto _ : state)
        benchmark::DoNotOptimize(CGAL::bounded_side_2(CGAL_bounds.begin(),
                             CGAL_bounds.end(),
                             Point(disRange(gen), disRange(gen)), K()));
}
BENCHMARK(BM_SearchMap_PointInPoly_Rand_CGAL_NoOpt);

class SearchMapFixture : public benchmark::Fixture {
    protected:
        SearchMap map;
        std::random_device rd;
        std::mt19937 gen;
    
    void SetUp(const ::benchmark::State& state) {
        gen = std::mt19937(rd());
        map.map_resolution = 10;
        map.x_start = 0;
        map.y_start = 0;
    }

    void TearDown(const ::benchmark::State& state) {
    }
};

BENCHMARK_DEFINE_F(SearchMapFixture, BM_SearchMap_Cell_Inside_xy_Poly)(benchmark::State& state) 
{
    std::uniform_real_distribution<> disRange(-200, 1200);

    std::vector<std::vector<double>> polygon_bounds = {{0, 0}, {0, 1000}, {1000, 1000}, {1000, 0}};
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(map.check_cell_inside_convex_poly((int) disRange(gen), (int) disRange(gen), polygon_bounds, state.range(0)));
        benchmark::ClobberMemory();
    }     
}
BENCHMARK_REGISTER_F(SearchMapFixture, BM_SearchMap_Cell_Inside_xy_Poly)->Arg(true)->Arg(false);
 
BENCHMARK_DEFINE_F(SearchMapFixture, BM_SearchMap_Cell_Inside_xy_Poly_Changing_Bounds)(benchmark::State& state) 
{
    std::uniform_real_distribution<> disRange(-200, 1200);
    std::uniform_real_distribution<> edgeRangeTop(900, 1100);
    std::uniform_real_distribution<> edgeRangeBottom(-100, 100);

    for (auto _ : state)
    {
        std::vector<std::vector<double>> polygon_bounds = {{edgeRangeBottom(gen), edgeRangeBottom(gen)}, 
                                                            {edgeRangeBottom(gen), edgeRangeTop(gen)}, 
                                                            {edgeRangeTop(gen), edgeRangeTop(gen)}, 
                                                            {edgeRangeTop(gen), edgeRangeBottom(gen)}};
        benchmark::DoNotOptimize(map.check_cell_inside_convex_poly((int) disRange(gen), (int) disRange(gen), polygon_bounds, state.range(0)));
        benchmark::ClobberMemory();
    }
}
BENCHMARK_REGISTER_F(SearchMapFixture, BM_SearchMap_Cell_Inside_xy_Poly_Changing_Bounds)->Arg(true)->Arg(false);
