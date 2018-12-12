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
        "Position X", "%6.2f", "m");

    timeseries_vehicleState[vehicle_id]["pose_y"] = make_shared<TimeSeries>(
        "Position Y", "%6.2f", "m");

    timeseries_vehicleState[vehicle_id]["pose_yaw"] = make_shared<TimeSeries>(
        "Yaw", "%6.3f", "rad");

    timeseries_vehicleState[vehicle_id]["odometer_distance"] = make_shared<TimeSeries>(
        "Odometer Distance", "%7.2f", "m");

    timeseries_vehicleState[vehicle_id]["imu_acceleration_forward"] = make_shared<TimeSeries>(
        "Acceleration Forward", "%4.1f", "m/s^2");

    timeseries_vehicleState[vehicle_id]["imu_acceleration_left"] = make_shared<TimeSeries>(
        "Acceleration Left", "%4.1f", "m/s^2");

    timeseries_vehicleState[vehicle_id]["speed"] = make_shared<TimeSeries>(
        "Speed", "%5.2f", "m/s");

    timeseries_vehicleState[vehicle_id]["battery_voltage"] = make_shared<TimeSeries>(
        "Battery Voltage", "%5.2f", "V");

    timeseries_vehicleState[vehicle_id]["motor_current"] = make_shared<TimeSeries>(
        "Motor Current", "%5.2f", "A");

    timeseries_vehicleState[vehicle_id]["clock_delta"] = make_shared<TimeSeries>(
        "Clock Delta", "%5.1f", "ms");
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

        timeseries_vehicleState[state.vehicle_id()]["clock_delta"]              ->push_sample(now, double(now - state.header().create_stamp().nanoseconds())/1e6 );


    }

}