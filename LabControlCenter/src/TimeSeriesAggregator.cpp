#include "TimeSeriesAggregator.hpp"

TimeSeriesAggregator::TimeSeriesAggregator(shared_ptr<dds::domain::DomainParticipant> _participant)
:participant(_participant)
{
    dds::topic::Topic<VehicleState> topic_VehicleState (*participant, "vehicleState");
    reader_VehicleState = make_shared<dds::sub::DataReader<VehicleState>>(dds::sub::Subscriber(*participant), topic_VehicleState);
    
    reader_VehicleState->listener(this, dds::core::status::StatusMask::data_available());

}


void TimeSeriesAggregator::create_vehicle_timeseries(uint8_t vehicle_id) 
{
    timeseries_vehicleState[vehicle_id] = map<string, shared_ptr<TimeSeries>>();

    timeseries_vehicleState[vehicle_id]["pose_x"] = make_shared<TimeSeries>(
        string_format("Vehicle %02u, Position X", vehicle_id), "%6.2f", "m");

    timeseries_vehicleState[vehicle_id]["pose_y"] = make_shared<TimeSeries>(
        string_format("Vehicle %02u, Position Y", vehicle_id), "%6.2f", "m");

    timeseries_vehicleState[vehicle_id]["pose_yaw"] = make_shared<TimeSeries>(
        string_format("Vehicle %02u, Yaw", vehicle_id), "%6.3f", "rad");

    timeseries_vehicleState[vehicle_id]["odometer_distance"] = make_shared<TimeSeries>(
        string_format("Vehicle %02u, Odometer Distance", vehicle_id), "%7.2f", "m");

    timeseries_vehicleState[vehicle_id]["imu_acceleration_forward"] = make_shared<TimeSeries>(
        string_format("Vehicle %02u, Acceleration Forward", vehicle_id), "%4.1f", "m/s^2");

    timeseries_vehicleState[vehicle_id]["imu_acceleration_left"] = make_shared<TimeSeries>(
        string_format("Vehicle %02u, Acceleration Left", vehicle_id), "%4.1f", "m/s^2");

    timeseries_vehicleState[vehicle_id]["speed"] = make_shared<TimeSeries>(
        string_format("Vehicle %02u, Speed", vehicle_id), "%5.2f", "m/s");

    timeseries_vehicleState[vehicle_id]["battery_voltage"] = make_shared<TimeSeries>(
        string_format("Vehicle %02u, Battery Voltage", vehicle_id), "%5.2f", "V");

    timeseries_vehicleState[vehicle_id]["motor_current"] = make_shared<TimeSeries>(
        string_format("Vehicle %02u, Motor Current", vehicle_id), "%5.2f", "A");

    for(auto callback : vehicle_added_callbacks)
    {
        if(callback)
        {
            callback(vehicle_id, timeseries_vehicleState);
        }
    }
}


void TimeSeriesAggregator::on_data_available(dds::sub::DataReader<VehicleState> &reader)
{
    const uint64_t now = clock_gettime_nanoseconds();

    vector<dds::sub::Sample<VehicleState>> new_states_sample;
    vector<VehicleState> new_states;
    reader.take(std::back_inserter(new_states_sample));
    for(auto state : new_states_sample)
    {
        try { new_states.push_back(state.data()); }
        catch(...){}
    }

    for(auto state : new_states) 
    {
        if(timeseries_vehicleState.count(state.vehicle_id()) == 0)
        {
            create_vehicle_timeseries(state.vehicle_id());
        }

        timeseries_vehicleState[state.vehicle_id()]["pose_x"]                   ->push_sample(now, state.pose().x());
        timeseries_vehicleState[state.vehicle_id()]["pose_y"]                   ->push_sample(now, state.pose().y());
        timeseries_vehicleState[state.vehicle_id()]["pose_yaw"]                 ->push_sample(now, state.pose().yaw());
        timeseries_vehicleState[state.vehicle_id()]["odometer_distance"]        ->push_sample(now, state.odometer_distance());
        timeseries_vehicleState[state.vehicle_id()]["imu_acceleration_forward"] ->push_sample(now, state.imu_acceleration_forward());
        timeseries_vehicleState[state.vehicle_id()]["imu_acceleration_left"]    ->push_sample(now, state.imu_acceleration_left());
        timeseries_vehicleState[state.vehicle_id()]["speed"]                    ->push_sample(now, state.speed());
        timeseries_vehicleState[state.vehicle_id()]["battery_voltage"]          ->push_sample(now, state.battery_voltage());
        timeseries_vehicleState[state.vehicle_id()]["motor_current"]            ->push_sample(now, state.motor_current());
    }

}