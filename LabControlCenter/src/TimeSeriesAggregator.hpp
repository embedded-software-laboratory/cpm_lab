#pragma once
#include "defaults.hpp"
#include <dds/sub/ddssub.hpp>
#include "VehicleState.hpp"
#include "VehicleObservation.hpp"
#include "TimeSeries.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "cpm/AsyncReader.hpp"
#include <mutex>


using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;
using VehicleTrajectories = map<uint8_t, shared_ptr<TimeSeries_TrajectoryPoint>  >;

class TimeSeriesAggregator
{
    VehicleData timeseries_vehicles;
    VehicleTrajectories vehicle_reference_trajectories;

    void create_vehicle_timeseries(uint8_t vehicle_id);
    void handle_new_vehicleState_samples(dds::sub::LoanedSamples<VehicleState>& samples);
    void handle_new_vehicleObservation_samples(dds::sub::LoanedSamples<VehicleObservation>& samples);
    void handle_new_commandTrajectory_samples(dds::sub::LoanedSamples<VehicleCommandTrajectory>& samples);

    shared_ptr<cpm::AsyncReader<VehicleState>> vehicle_state_reader;
    shared_ptr<cpm::AsyncReader<VehicleObservation>> vehicle_observation_reader;
    shared_ptr<cpm::AsyncReader<VehicleCommandTrajectory>> vehicle_commandTrajectory_reader;

    std::mutex _mutex;

public:
    TimeSeriesAggregator();
    VehicleData get_vehicle_data();
    VehicleTrajectories get_vehicle_trajectory_commands();
};