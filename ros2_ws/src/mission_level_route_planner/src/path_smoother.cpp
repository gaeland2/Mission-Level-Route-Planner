#include "mission_level_route_planner/path_smoother.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>

namespace mission_level_route_planner
{

    PathSmoother::PathSmoother(std::shared_ptr<OccupancyGrid> grid)
    : grid_(std::move(grid))
    {
        if (!grid_) {
            throw std::invalid_argument("PathSmoother requires a valid occupancy grid.");
        }
    }

    double PathSmoother::path_length(const std::vector<Index> & path) const
    {
        double total = 0.0;

        for (size_t i = 1; i < path.size(); ++i) {
            const auto & a = path[i - 1];
            const auto & b = path[i];

            const double dx = static_cast<double>(b[0] - a[0]);
            const double dy = static_cast<double>(b[1] - a[1]);
            const double dz = static_cast<double>(b[2] - a[2]);

            total += std::sqrt(dx * dx + dy * dy + dz * dz) * grid_->resolution();
        }

        return total;
    }

    bool PathSmoother::line_of_sight(
        const Index & start, const Index & end) const
    {
        if (!grid_->is_free(start) || !grid_->is_free(end)) {
            return false;
        }

        // Work in grid coordinates, where each cell has width 1.
        // The segment connects the centers of the start and end cells.
        const double dx = static_cast<double>(end[0]) - start[0];
        const double dy = static_cast<double>(end[1]) - start[1];
        const double dz = static_cast<double>(end[2]) - start[2];

        const double delta[3] = {dx, dy, dz};

        Index current = start;
        int direction[3] = {0, 0, 0};
        double t_max[3];
        double t_delta[3];

        const double infinity = std::numeric_limits<double>::infinity();

        for (int axis = 0; axis < 3; ++axis) {
            if (delta[axis] > 0.0) {
                direction[axis] = 1;
            } else if (delta[axis] < 0.0) {
                direction[axis] = -1;
            }

            if (direction[axis] == 0) {
                t_max[axis] = infinity;
                t_delta[axis] = infinity;
            } else {
                t_max[axis] = 0.5 / std::abs(delta[axis]);
                t_delta[axis] = 1.0 / std::abs(delta[axis]);
            }
        }

        while (current != end) {
            const double next_t = std::min({t_max[0], t_max[1], t_max[2]});

            // Identify every axis crossed at the same point.
            bool crossing[3] = {
                std::abs(t_max[0] - next_t) < 1e-12,
                std::abs(t_max[1] - next_t) < 1e-12,
                std::abs(t_max[2] - next_t) < 1e-12
            };

            // Check all cells touched at an edge or corner.
            for (int mask = 1; mask < 8; ++mask) {
                Index neighbor = current;
                bool valid = true;

                for (int axis = 0; axis < 3; ++axis) {
                    if (mask & (1 << axis)) {
                        if (!crossing[axis]) {
                            valid = false;
                            break;
                        }
                        neighbor[axis] += direction[axis];
                    }
                }

                if (valid && !grid_->is_free(neighbor)) {
                    return false;
                }
            }

            // Advance along all axes crossed at this point.
            for (int axis = 0; axis < 3; ++axis) {
                if (crossing[axis]) {
                    current[axis] += direction[axis];
                    t_max[axis] += t_delta[axis];
                }
            }
        }

        return true;
    }
    
    std::vector<Index> PathSmoother::simplify(
        const std::vector<Index> & path,
        double length_tolerance) const
        {
            if (length_tolerance < 0.0 || !std::isfinite(length_tolerance)) {
                throw std::invalid_argument("Length tolerance must be finite and non-negative.");
            }

            // Check every original path segment for collisions.
            for (size_t i = 1; i < path.size(); ++i) {
                if (!line_of_sight(path[i - 1], path[i])) {
                    throw std::invalid_argument(
                    "Original path contains a collision.");
                }
            }

            // Paths with two or fewer waypoints cannot be simplified further.
            if (path.size() <= 2) {
                return path;
            }

            std::vector<Index> simplified;
            simplified.push_back(path.front());

            size_t current = 0;

            while (current < path.size() - 1) {
                size_t next = current + 1;

                // Find the farthest waypoint reachable in a straight line.
                for (size_t candidate = path.size() - 1; candidate > current + 1; --candidate) {
                    if (line_of_sight(path[current], path[candidate])) {
                        next = candidate;
                        break;
                    }
                }
                simplified.push_back(path[next]);
                current = next;
            }

            // Keep the original path if the new path exceeds the length tolerance.
            const double original_length = path_length(path);
            const double simplified_length = path_length(simplified);

            if (simplified_length > original_length * (1.0 + length_tolerance)) {
                return path;
            }

            return simplified;
        }
}  // namespace mission_level_route_planner