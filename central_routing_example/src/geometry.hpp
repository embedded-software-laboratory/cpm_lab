#pragma once
#include <vector>
#include <cassert>
#include <cmath>
using std::vector;

#define VEHICLE_HALF_LENGTH (0.12)
#define VEHICLE_HALF_WDITH (0.06)

static inline double min_distance_vehicle_to_points
(
    double vehicle_x,
    double vehicle_y,
    double vehicle_cos_yaw,
    double vehicle_sin_yaw,
    const vector<double> &points_x,
    const vector<double> &points_y
)
{
    assert(points_x.size() == points_y.size());
    double min_distance = 1e300;

    for (size_t i = 0; i < points_x.size(); ++i)
    {
        const double dx = vehicle_x - points_x[i];
        const double dy = vehicle_y - points_y[i];

        const double dist_longitudinal = fmax(fabs(dx * vehicle_cos_yaw + dy * vehicle_sin_yaw) - VEHICLE_HALF_LENGTH, 0);
        const double dist_lateral      = fmax(fabs(dy * vehicle_cos_yaw - dx * vehicle_sin_yaw) - VEHICLE_HALF_WDITH, 0);

        min_distance = fmin(min_distance, fmax(dist_longitudinal, dist_lateral));
    }

    return min_distance;
}