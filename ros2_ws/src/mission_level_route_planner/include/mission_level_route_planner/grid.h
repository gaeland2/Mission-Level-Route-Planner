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
  OccupancyGrid(int nx, int ny, int nz, double resolution = 1.0, Waypoint origin = Waypoint{});

  bool in_bounds(const Index & n) const;

  //NOTE : free = inside map, inside altitude limit, and not a blocked cell
  bool is_free(const Index & n) const;
  void block(const Index & n);
  void unblock(const Index & n);


  /*
    here, the altitude limit can be set (the values min_z_m and max_z_m are in METERS, z = up.
    Cell should be allowed when the center height is within range, including the cells at the range's ends

    Cells outside of range are NOT FREE, so the planner and start/goal validation should also respect the limit

    catch invalid arg if min > max
  */
  void set_altitude_limits(double min_z_m, double max_z_m);
  void clear_altitude_limits();
  bool within_altitude_limits(const Index & n) const;

  // Free neighbors of n. connectivity is 6 (faces), 18 (faces and edges) or 26 (all).
  vector<Index> neighbors(const Index & n, int connectivity = 6) const;

  // Distance in meters between two neighboring cells.
  double step_cost(const Index & a, const Index & b) const;

  // Flat id <-> index, so planners can use plain vectors instead of hash maps.

  //if we prefer hashmaps let me know
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
  vector<uint8_t> blocked_;
};


} // end namespace
