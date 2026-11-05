// Sprint 1 demo: plan across a small hand-made map and print the result. No ROS needed.
#include <cstdio>
#include <memory>

#include "mission_level_route_planner/context.h"
#include "mission_level_route_planner/dijkstra.h"

using namespace std;

using namespace mission_level_route_planner;  // NOLINT

int main()
{
  auto grid = make_shared<OccupancyGrid>(20, 20, 1, 1.0);
  for (int y = 0; y < 15; ++y) {grid->block({10, y, 0});}  // wall at x = 10, gap at the top

  Context ctx(grid, make_shared<Dijkstra>(grid, 18));
  const PlanResult r = ctx.plan({2, 2, 0}, {17, 2, 0});

  printf(
    "success=%d length=%.2f m time=%.2f ms expanded=%d\n", r.success(), r.length_m,
    r.compute_time_s * 1000.0, r.nodes_expanded);
  for (const auto & n : r.path) {printf("(%d, %d, %d)\n", n[0], n[1], n[2]);}
  return r.success() ? 0 : 1;
}
