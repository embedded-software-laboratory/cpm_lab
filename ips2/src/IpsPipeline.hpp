#pragma once
#include "types.hpp"
#include "UndistortPoints.hpp"
#include "LedPoints.hpp"
#include "DetectVehicles.hpp"
#include "DetectVehicleID.hpp"
#include "PoseCalculation.hpp"
#include <memory>
#include <mutex>
#include <thread>
#include <dds/pub/ddspub.hpp>

struct IpsVisualizationInput
{
    VehiclePoints identifiedVehicles;
    std::vector<VehicleObservation> vehicleObservations;
    FloorPoints floorPoints;
    VehiclePoints vehiclePoints;
};

class IpsPipeline
{
    dds::pub::DataWriter<VehicleObservation> writer_vehicleObservation;

    std::shared_ptr<UndistortPoints> undistortPointsFn;
    std::shared_ptr<DetectVehicles> detectVehiclesFn;
    std::shared_ptr<DetectVehicleID> detectVehicleIDfn;
    std::shared_ptr<PoseCalculation> poseCalculationFn;

    VehiclePointTimeseries vehiclePointTimeseries;


    // Temporary copy, for the visualization in another thread
    IpsVisualizationInput ipsVisualizationInput_buffer;
    std::mutex ipsVisualizationInput_buffer_mutex;
    std::thread visualization_thread;

public:
    IpsPipeline();
    void apply(LedPoints led_points);
    cv::Mat visualization(const IpsVisualizationInput &input);
    void visualization_loop();
    
    
};