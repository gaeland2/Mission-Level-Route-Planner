#include "mission_level_route_planner/dijkstra.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

using namespace std;

namespace mission_level_route_planner
{

Dijkstra::Dijkstra(shared_ptr<OccupancyGrid> grid, int connectivity)
: grid_(move(grid)), connectivity_(connectivity)
{
}

PlanResult Dijkstra::execute_pathfinding(const Index & start, const Index & goal)
{
  const auto t0 = chrono::steady_clock::now();
  const auto elapsed = [&t0]() {
      return chrono::duration<double>(chrono::steady_clock::now() - t0).count();
    };

  const double inf = numeric_limits<double>::infinity();
  const int count = grid_->cell_count();
  vector<double> dist(count, inf);
  vector<int> parent(count, -1);
  vector<char> done(count, 0);

  using QItem = pair<double, int>;  // distance, cell id
  priority_queue<QItem, vector<QItem>, greater<QItem>> open;

  const int start_id = grid_->to_id(start);
  const int goal_id = grid_->to_id(goal);
  dist[start_id] = 0.0;
  open.emplace(0.0, start_id);

  PlanResult result;
  while (!open.empty()) {
    const auto [d, id] = open.top();
    open.pop();
    if (done[id]) {continue;}  // stale queue entry
    done[id] = 1;
    ++result.nodes_expanded;

    if (id == goal_id) {
      for (int at = goal_id; at != -1; at = parent[at]) {
        result.path.push_back(grid_->from_id(at));
      }
      reverse(result.path.begin(), result.path.end());
      result.status = PlanStatus::OK;
      result.length_m = d;
      result.compute_time_s = elapsed();
      return result;
    }

    const Index node = grid_->from_id(id);
    for (const Index & next : grid_->neighbors(node, connectivity_)) {
      const int next_id = grid_->to_id(next);
      const double nd = d + grid_->step_cost(node, next);
      if (nd < dist[next_id]) {
        dist[next_id] = nd;
        parent[next_id] = id;
        open.emplace(nd, next_id);
      }
    }
  }

  // Open set is empty and the goal was never reached.
  result.status = PlanStatus::NO_PATH;
  result.compute_time_s = elapsed();
  return result;
}

}  // namespace mission_level_route_planner
