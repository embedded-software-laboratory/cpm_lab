#pragma once
#include "defaults.hpp"
#include <dds/sub/ddssub.hpp>
#include "VehicleState.hpp"
#include "TimeSeries.hpp"


class TimeSeriesAggregator : dds::sub::NoOpDataReaderListener<VehicleState>
{
    shared_ptr<dds::domain::DomainParticipant> participant;
    shared_ptr<dds::sub::DataReader<VehicleState>> reader_VehicleState;
    map<uint8_t, map<string, shared_ptr<TimeSeries> > > timeseries_vehicleState;
    void create_vehicle_timeseries(uint8_t vehicle_id);
    void on_data_available (dds::sub::DataReader<VehicleState>&) override;


    vector<std::function<void(uint8_t new_vehicle_id, map<uint8_t, map<string, shared_ptr<TimeSeries> > >& timeseries)>> vehicle_added_callbacks;

public:
    TimeSeriesAggregator(shared_ptr<dds::domain::DomainParticipant>);  

    void add_vehicle_added_callback(std::function<void(uint8_t new_vehicle_id, map<uint8_t, map<string, shared_ptr<TimeSeries> > >& timeseries)> callback)
    { vehicle_added_callbacks.push_back(callback); } 
};