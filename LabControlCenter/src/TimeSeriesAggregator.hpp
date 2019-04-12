#pragma once
#include "defaults.hpp"
#include <dds/sub/ddssub.hpp>
#include "VehicleState.hpp"
#include "VehicleObservation.hpp"
#include "TimeSeries.hpp"
#include "cpm/AsyncReader.hpp"
#include <mutex>


using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;

class TimeSeriesAggregator
{
    map<uint8_t, map<string, shared_ptr<TimeSeries> > > timeseries_vehicles;
    void create_vehicle_timeseries(uint8_t vehicle_id);
    void handle_new_vehicleState_samples(dds::sub::LoanedSamples<VehicleState>& samples);
    void handle_new_vehicleObservation_samples(dds::sub::LoanedSamples<VehicleObservation>& samples);

    shared_ptr<cpm::AsyncReader<VehicleState>> vehicle_state_reader;
    shared_ptr<cpm::AsyncReader<VehicleObservation>> vehicle_observation_reader;

    std::mutex _mutex;

public:
    TimeSeriesAggregator();
    VehicleData get_vehicle_data();
};