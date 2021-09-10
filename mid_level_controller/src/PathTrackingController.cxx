#include "PathTrackingController.hpp"
#include <cassert>
#include <cmath>
#include "PathInterpolation.hpp"

PathTrackingController::PathTrackingController(uint8_t vehicle_id)
    :writer_Visualization("visualization")
    ,vehicle_id(vehicle_id)
{}


// Wrap angle in radians to [-pi pi]
inline double wrap2pi(const double yaw)
{
    double yaw_out = fmod(yaw + M_PI, 2.0*M_PI);
    if (yaw_out < 0) yaw_out += 2.0*M_PI;
    return yaw_out - M_PI;
}

double PathTrackingController::control_steering_servo(
        const VehicleState &vehicleState,                  
        const VehicleCommandPathTracking &commandPathTracking
)
{
    // Compute front axle position
    double x   = vehicleState.pose().x();
    double y   = vehicleState.pose().y();
    double yaw = vehicleState.pose().yaw();

    const double wheelbase = 0.15; // m
    double x_f = x + wheelbase/2*cos(yaw);
    double y_f = y + wheelbase/2*sin(yaw);

    // Find reference point on path
    std::vector<PathPoint> path = commandPathTracking.path();
    Pose2D ref_pose = find_reference_pose(path, x_f, y_f);


    // compute control errors
    double tHat_x = cos(ref_pose.yaw());
    double tHat_y = sin(ref_pose.yaw());
    double d_x = x_f - ref_pose.x();
    double d_y = y_f - ref_pose.y();
    
    double e = d_x*tHat_y - d_y*tHat_x;
    double psi = wrap2pi(ref_pose.yaw() - yaw);


    // send reference pose as virtualisation
    Visualization vis;
    vis.id(vehicle_id);
    vis.type(VisualizationType::LineStrips);
    vis.time_to_live(25000000ull);
    vis.size(0.03);
    vis.color().r(0);
    vis.color().g(204);
    vis.color().b(204);

    const double vis_length = 0.12;

    vis.points().resize(2);
    vis.points().at(0).x(ref_pose.x());
    vis.points().at(0).y(ref_pose.y());
    vis.points().at(1).x(ref_pose.x() + tHat_y * vis_length);
    vis.points().at(1).y(ref_pose.y() + -tHat_x * vis_length);

    writer_Visualization.write(vis);


    // control law for steering servo
    double k = 5;
    double k_soft = 0.3;
    double delta = psi 
        + atan( k*e / (k_soft+vehicleState.speed()) );
        // + k_dsteer * (delta_last(1) - delta_last(2));
        // - k_dyaw*(dyaw - path.k(i_path_ref)*vehicleState.speed()) ...
        
    // tune down aggressiveness at fast speeds
    delta = delta / ( (0.25*vehicleState.speed()*vehicleState.speed()) + 1 );
    
    double steering_servo = 0.226*(exp(3.509*delta)-exp(-3.509*delta));
    return steering_servo;
}

Pose2D PathTrackingController::find_reference_pose(
    const std::vector<PathPoint> &path,
    const double x,
    const double y
)
{
    // find closest junction point
    assert(path.size() > 1);
    size_t i_junction_closest = 0;

    double min_dist = 1e300;
    // first and last point should be equal, so size()-1
    for (size_t i_junction = 0; i_junction < path.size()-1; ++i_junction)
    {
        double dx = x - path[i_junction].pose().x();
        double dy = y - path[i_junction].pose().y();
        double distance = sqrt( dx*dx + dy*dy );

        if (distance < min_dist)
        {
            min_dist = distance;
            i_junction_closest = i_junction;
        }
    }

    // consider path segment leading to and leaving junction
    std::vector<size_t> i_start;
    i_start.push_back(i_junction_closest);
    if (i_junction_closest == 0)
    {
        i_start.push_back(path.size()-2);
    }
    else
    {
        i_start.push_back(i_junction_closest-1);
    }
    
    
    // Iterate over interpolated path to find point with minimum distance
    const double ds = 0.01; // [m]
    
    min_dist = 1e300;
    Pose2D result;
    for (size_t i_path_point : i_start)
    {
        double s_start = path[i_path_point].s();
        double s_end = path[i_path_point+1].s();
        for (double s_query = s_start; s_query <= s_end; s_query += ds)
        {
            // calculate distance to reference path
            PathInterpolation path_interpolation(
                s_query, path[i_path_point], path[i_path_point+1]
            );
            double dx = x - path_interpolation.position_x;
            double dy = y - path_interpolation.position_y;
            double distance = sqrt( dx*dx + dy*dy );

            if (distance < min_dist)
            {
                min_dist = distance;
                // update reference pose
                result.x(path_interpolation.position_x);
                result.y(path_interpolation.position_y);
                result.yaw(path_interpolation.yaw);
            }
        }
    }
    
    return result;
}