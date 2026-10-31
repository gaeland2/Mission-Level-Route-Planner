//3D occupancy grid (header). OctomapAdapter should build off of this from the .bt file

#pragma once

#include <cstdint>
#include <vector>

#include "mission_level_route_planner/models.h" //use some parts from models header

using namespace std;

namespace mission_level_route_planner {

// cells should be free unless blocked, and occupied cells should be impassable
class OccupancyGrid {
public:
  OccupancyGrid(
    int nx, int ny, int nz, double resolution = 1.0,
    Waypoint origin = Waypoint{});

  bool in_bounds(const Index & n) const;
  // Free means: inside the map, inside the altitude limits, and not covered by an obstacle or
  // by an obstacle's safety margin.
  bool is_free(const Index & n) const;
  // Mark / unmark a real obstacle. The safety margin around it follows automatically.
  void block(const Index & n);
  void unblock(const Index & n);
  // True only for cells that were blocked as real obstacles, ignoring the safety margin.
  bool is_occupied(const Index & n) const;

  // Keep the drone at least radius_m meters (cell center to cell center) away from every
  // obstacle. The margin is a sphere, so it is the same in all three directions. It is applied
  // to obstacles outside the altitude limits too. 0 turns it off.
  // Throws invalid_argument if radius_m is negative or so large that it is not supported.
  void set_safety_margin(double radius_m);
  double safety_margin() const {return margin_m_;}

  // Restrict flying to min_z_m..max_z_m (meters in the map frame, z up). A cell is allowed
  // when its center height is inside that range, ends included. Cells outside the range
  // count as not free, so every planner and the start/goal validation respect the limits.
  // Throws invalid_argument if min_z_m > max_z_m.
  void set_altitude_limits(double min_z_m, double max_z_m);
  void clear_altitude_limits();
  bool within_altitude_limits(const Index & n) const;

  // Free neighbors of n. connectivity is 6 (faces), 18 (faces and edges) or 26 (all).
  vector<Index> neighbors(const Index & n, int connectivity = 6) const;

  // Distance in meters between two neighboring cells.
  double step_cost(const Index & a, const Index & b) const;

  // Flat id <-> index, so planners can use plain vectors instead of hash maps.
  int cell_count() const {return nx_ * ny_ * nz_;}
  int to_id(const Index & n) const {return (n[0] * ny_ + n[1]) * nz_ + n[2];}
  Index from_id(int id) const;

  Waypoint to_world(const Index & n) const;   // center of the cell
  Index to_index(const Waypoint & p) const;

  double resolution() const {return resolution_;}

private:
  int nx_, ny_, nz_;
  double resolution_;
  Waypoint origin_;
  int k_min_;  // lowest allowed z layer
  int k_max_;  // highest allowed z layer (k_min_ > k_max_ means no layer is allowed)
  // Adds delta (+1 or -1) to the cover count of every cell in the sphere around n.
  void apply_margin(const Index & n, int delta);

  double margin_m_;
  vector<Index> margin_offsets_;  // cell offsets inside the sphere; {0,0,0} when margin is 0
  vector<uint8_t> occupied_;      // 1 = real obstacle
  vector<uint16_t> cover_;        // how many obstacles' spheres contain this cell
};

}  // namespace mission_level_route_planner

