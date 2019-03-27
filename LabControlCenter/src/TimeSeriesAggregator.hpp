#pragma once
#include "defaults.hpp"
#include <dds/sub/ddssub.hpp>
#include "VehicleState.hpp"
#include "TimeSeries.hpp"
#include "cpm/AsyncReader.hpp"

class TimeSeriesAggregator
{
    map<uint8_t, map<string, shared_ptr<TimeSeries> > > timeseries_vehicleState;
    void create_vehicle_timeseries(uint8_t vehicle_id);
    void handle_new_vehicleState_samples(dds::sub::LoanedSamples<VehicleState>& samples);

    shared_ptr<cpm::AsyncReader<VehicleState>> vehicle_state_reader;

public:
    TimeSeriesAggregator();
    const map<uint8_t, map<string, shared_ptr<TimeSeries> > >& get_vehicle_data() { return timeseries_vehicleState; }
};