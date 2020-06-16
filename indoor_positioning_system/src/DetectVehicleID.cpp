#include "DetectVehicleID.hpp"
#include <iostream>

DetectVehicleID::DetectVehicleID(
    std::vector<uint8_t> _identification_LED_period_ticks,
    std::vector<uint8_t> _identification_LED_enabled_ticks
)
:identification_LED_period_ticks(_identification_LED_period_ticks)
,identification_LED_enabled_ticks(_identification_LED_enabled_ticks)
{
    assert(identification_LED_enabled_ticks.size() == identification_LED_period_ticks.size());
}

struct VehicleTrackingInfo
{
    bool center_present = false;
    cv::Point2d centroid;
};

struct SignalEdge
{
    int position;
    bool is_rising;
};

VehiclePoints DetectVehicleID::apply(const VehiclePointTimeseries &vehiclePointTimeseries)
{
    assert(vehiclePointTimeseries.size() > 30);

    // We want to identify the vehicles in the most recent frame: vehiclePointTimeseries.back().
    // Algorithm: Iterate backwards in time, and track which vehicle is
    // which by distance. A vehicle can move no more than 0.1 meters per frame.
    // Then for each vehicle, count the consecutive frames where the center LED is on/off.
    // The pair of (on/off) counts maps to a vehicle ID.



    // This will hold a sort-of transpose of the vehilces over time: 
    // The outer index corresponds to a vehicle.
    // The inner index corresponds to a point in time, in reverse chronological order.
    std::vector< std::vector< VehicleTrackingInfo > > vehicles_tracked_by_proximity;


    // Add the most recent vehicle. This is the starting point for the tracking.
    auto iter = vehiclePointTimeseries.rbegin();
    for(VehiclePointSet vehicle : iter->vehicles)
    {
        VehicleTrackingInfo vehicleTrackingInfo;
        vehicleTrackingInfo.centroid = (1.0/3) * 
            (vehicle.front + vehicle.back_left + vehicle.back_right);
        vehicleTrackingInfo.center_present = vehicle.center_present;

        vehicles_tracked_by_proximity.push_back(
            std::vector< VehicleTrackingInfo >{vehicleTrackingInfo}
        );
    }

    // Iterate backwards in time. Track vehicles by distance.
    ++iter;
    for (size_t step_count = 1; iter != vehiclePointTimeseries.rend(); ++iter, ++step_count)
    {
        // For each tracked vehicle, find the corresponding vehicle in the previous frame.
        for(auto &tracked_vehicle : vehicles_tracked_by_proximity)
        {
            // Check if the tracking has been lost.
            // This happens when a vehicle is not detected in a frame.
            if(tracked_vehicle.size() == step_count)
            {
                // Search for the vehicle, which corresponds to "tracked_vehicle".
                for(VehiclePointSet previous_vehicle : iter->vehicles)
                {
                    const cv::Point2d centroid = (1.0/3) * (previous_vehicle.front + previous_vehicle.back_left + previous_vehicle.back_right);
                    const auto delta = tracked_vehicle.back().centroid - centroid;
                    const double max_distance_per_frame_squared = 0.09 * 0.09;

                    if(delta.dot(delta) < max_distance_per_frame_squared)
                    {
                        // Matching vehicle found, add to tracking.
                        VehicleTrackingInfo vehicleTrackingInfo;
                        vehicleTrackingInfo.centroid = centroid;
                        vehicleTrackingInfo.center_present = previous_vehicle.center_present;
                        tracked_vehicle.push_back(vehicleTrackingInfo);
                        break;
                    }
                }
            }
            else
            {
                // Tracking is lost. Do nothing.
            }
        }
    } 


    // The vehicles are now tracked, each vehicles_tracked_by_proximity[i]
    // corresponds to a particular physical vehicle.
    // Now extract the ID of each vehicle.

    VehiclePoints result = vehiclePointTimeseries.back();

    for (size_t vehicle_index = 0; vehicle_index < vehicles_tracked_by_proximity.size(); ++vehicle_index)
    {
        auto &vehicle = vehicles_tracked_by_proximity[vehicle_index];

        // Find signal edges
        std::vector<SignalEdge> signal_edges;
        for (size_t i = 0; i < vehicle.size()-1; ++i)
        {
            if(!(vehicle.at(i).center_present) && (vehicle.at(i+1).center_present))
            {
                SignalEdge edge;
                edge.position = i;
                edge.is_rising = true;
                //std::cout << "rising " << i << std::endl;
                signal_edges.push_back(edge);
            }
            else if((vehicle.at(i).center_present) && !(vehicle.at(i+1).center_present))
            {
                SignalEdge edge;
                edge.position = i;
                edge.is_rising = false;
                //std::cout << "falling " << i << std::endl;
                signal_edges.push_back(edge);
            }
        }

        // Find vehicle ID, based on signal edges
        if(signal_edges.size() >= 3)
        {
            int total_frames = signal_edges[2].position - signal_edges[0].position;
            int high_frames = 0;

            if(signal_edges[0].is_rising)
            {
                high_frames = signal_edges[1].position - signal_edges[0].position;
            }
            else
            {
                high_frames = signal_edges[2].position - signal_edges[1].position;
            }

            for (size_t vehicle_id = 1; vehicle_id < identification_LED_enabled_ticks.size(); ++vehicle_id)
            {
                int delta_high = high_frames - identification_LED_enabled_ticks.at(vehicle_id);
                int delta_total = total_frames - identification_LED_period_ticks.at(vehicle_id);

                if(    -1 <= delta_high && delta_high <= 1
                    && -1 <= delta_total && delta_total <= 1)
                {
                    result.vehicles.at(vehicle_index).id = vehicle_id;
                    break;
                }
            }
        }
    }

    return result;
}

