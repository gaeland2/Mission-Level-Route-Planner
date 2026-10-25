#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace std;

namespace mission_level_route_planner // use the namespace so our code doesnt conflict with ROS2/Octomap/MAVROS
{


//full OccupancyGrid usage goes here
OccupancyGrid::OccupancyGrid(int nx, int ny, int nz, double resolution, Waypoint origin)
: nx_(nx), ny_(ny), nz_(nz), resolution_(resolution), origin_(origin),
  k_min_(0), k_max_(nz - 1),
  blocked_(static_cast<size_t>(nx) * ny * nz, 0)
{
}


//OccupancyGrid functions here
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
  return in_bounds(n) && within_altitude_limits(n) && blocked_[to_id(n)] == 0;
}

void OccupancyGrid::block(const Index & n)
{
  if (in_bounds(n)) {blocked_[to_id(n)] = 1;}
}

void OccupancyGrid::unblock(const Index & n)
{
  if (in_bounds(n)) {blocked_[to_id(n)] = 0;}
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




} // end of namespace
