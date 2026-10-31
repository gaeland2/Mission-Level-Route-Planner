#include "mission_level_route_planner/grid.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace std;

namespace mission_level_route_planner
{

OccupancyGrid::OccupancyGrid(int nx, int ny, int nz, double resolution, Waypoint origin)
: nx_(nx), ny_(ny), nz_(nz), resolution_(resolution), origin_(origin),
  k_min_(0), k_max_(nz - 1),
  margin_m_(0.0), margin_offsets_{Index{0, 0, 0}},
  occupied_(static_cast<size_t>(nx) * ny * nz, 0),
  cover_(static_cast<size_t>(nx) * ny * nz, 0)
{
}

void OccupancyGrid::set_safety_margin(double radius_m)
{
  if (radius_m < 0.0) {
    throw invalid_argument("safety margin must not be negative");
  }
  // Every cell offset whose center is within radius_m of the obstacle cell's center.
  const double eps = 1e-9;
  const double reach_cells = floor(radius_m / resolution_ + eps);
  // A sphere with a radius of more than 26 cells has more than 65535 cells, so reject it
  // before looping over it.
  if (reach_cells > 26.0) {
    throw invalid_argument("safety margin is too large for this grid resolution");
  }
  const int reach = static_cast<int>(reach_cells);
  vector<Index> offsets;
  for (int di = -reach; di <= reach; ++di) {
    for (int dj = -reach; dj <= reach; ++dj) {
      for (int dk = -reach; dk <= reach; ++dk) {
        const double d = resolution_ * sqrt(static_cast<double>(di * di + dj * dj + dk * dk));
        if (d <= radius_m + eps) {offsets.push_back(Index{di, dj, dk});}
      }
    }
  }
  // cover_ counts are 16-bit, so a cell can be covered by at most 65535 obstacles.
  if (offsets.size() > 65535) {
    throw invalid_argument("safety margin is too large for this grid resolution");
  }
  margin_m_ = radius_m;
  margin_offsets_ = move(offsets);

  // Rebuild the cover counts from the real obstacles.
  fill(cover_.begin(), cover_.end(), 0);
  for (int id = 0; id < cell_count(); ++id) {
    if (occupied_[id]) {apply_margin(from_id(id), +1);}
  }
}

void OccupancyGrid::apply_margin(const Index & n, int delta)
{
  for (const Index & o : margin_offsets_) {
    const Index m{n[0] + o[0], n[1] + o[1], n[2] + o[2]};
    if (in_bounds(m)) {cover_[to_id(m)] += delta;}
  }
}

void OccupancyGrid::set_altitude_limits(double min_z_m, double max_z_m)
{
  if (min_z_m > max_z_m) {
    throw invalid_argument("min altitude must not be above max altitude");
  }
  // Layer k has its center at origin.z + (k + 0.5) * resolution. Keep layers whose center is in
  // [min, max]. The small epsilon stops a limit that lands exactly on a center from being lost
  // to floating-point error.
  const double eps = 1e-9;
  const double lo = ceil((min_z_m - origin_.z) / resolution_ - 0.5 - eps);
  const double hi = floor((max_z_m - origin_.z) / resolution_ - 0.5 + eps);
  k_min_ = static_cast<int>(min(max(lo, 0.0), static_cast<double>(nz_)));
  k_max_ = static_cast<int>(max(min(hi, nz_ - 1.0), -1.0));
}

void OccupancyGrid::clear_altitude_limits()
{
  k_min_ = 0;
  k_max_ = nz_ - 1;
}

bool OccupancyGrid::within_altitude_limits(const Index & n) const
{
  return n[2] >= k_min_ && n[2] <= k_max_;
}

bool OccupancyGrid::in_bounds(const Index & n) const
{
  return n[0] >= 0 && n[0] < nx_ && n[1] >= 0 && n[1] < ny_ && n[2] >= 0 && n[2] < nz_;
}

bool OccupancyGrid::is_free(const Index & n) const
{
  return in_bounds(n) && within_altitude_limits(n) && cover_[to_id(n)] == 0;
}

bool OccupancyGrid::is_occupied(const Index & n) const
{
  return in_bounds(n) && occupied_[to_id(n)] != 0;
}

void OccupancyGrid::block(const Index & n)
{
  if (!in_bounds(n) || occupied_[to_id(n)]) {return;}
  occupied_[to_id(n)] = 1;
  apply_margin(n, +1);
}

void OccupancyGrid::unblock(const Index & n)
{
  if (!in_bounds(n) || !occupied_[to_id(n)]) {return;}
  occupied_[to_id(n)] = 0;
  apply_margin(n, -1);
}

vector<Index> OccupancyGrid::neighbors(const Index & n, int connectivity) const
{
  vector<Index> out;
  out.reserve(26);
  for (int di = -1; di <= 1; ++di) {
    for (int dj = -1; dj <= 1; ++dj) {
      for (int dk = -1; dk <= 1; ++dk) {
        const int steps = (di != 0) + (dj != 0) + (dk != 0);
        if (steps == 0) {continue;}
        if (connectivity == 6 && steps > 1) {continue;}
        if (connectivity == 18 && steps > 2) {continue;}
        const Index m{n[0] + di, n[1] + dj, n[2] + dk};
        if (is_free(m)) {out.push_back(m);}
      }
    }
  }
  return out;
}

double OccupancyGrid::step_cost(const Index & a, const Index & b) const
{
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  const double dz = a[2] - b[2];
  return resolution_ * sqrt(dx * dx + dy * dy + dz * dz);
}

Index OccupancyGrid::from_id(int id) const
{
  const int k = id % nz_;
  const int j = (id / nz_) % ny_;
  const int i = id / (nz_ * ny_);
  return Index{i, j, k};
}

Waypoint OccupancyGrid::to_world(const Index & n) const
{
  return Waypoint{
    origin_.x + (n[0] + 0.5) * resolution_,
    origin_.y + (n[1] + 0.5) * resolution_,
    origin_.z + (n[2] + 0.5) * resolution_};
}

Index OccupancyGrid::to_index(const Waypoint & p) const
{
  return Index{
    static_cast<int>(floor((p.x - origin_.x) / resolution_)),
    static_cast<int>(floor((p.y - origin_.y) / resolution_)),
    static_cast<int>(floor((p.z - origin_.z) / resolution_))};
}

}  // namespace mission_level_route_planner
