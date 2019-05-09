#include "TimeSeriesAggregator.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/ParticipantSingleton.hpp"

TimeSeriesAggregator::TimeSeriesAggregator()
{
    vehicle_state_reader = make_shared<cpm::AsyncReader<VehicleState>>(
        [this](dds::sub::LoanedSamples<VehicleState>& samples){
            handle_new_vehicleState_samples(samples);
        },
        cpm::ParticipantSingleton::Instance(),
        cpm::get_topic<VehicleState>("vehicleState")
    );


    vehicle_observation_reader = make_shared<cpm::AsyncReader<VehicleObservation>>(
        [this](dds::sub::LoanedSamples<VehicleObservation>& samples){
            handle_new_vehicleObservation_samples(samples);
        },
        cpm::ParticipantSingleton::Instance(),
        cpm::get_topic<VehicleObservation>("vehicleObservation")
    );


    vehicle_commandTrajectory_reader = make_shared<cpm::AsyncReader<VehicleCommandTrajectory>>(
        [this](dds::sub::LoanedSamples<VehicleCommandTrajectory>& samples){
            //handle_new_commandTrajectory_samples(samples);
        },
        cpm::ParticipantSingleton::Instance(),
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory")
    );
}


void TimeSeriesAggregator::create_vehicle_timeseries(uint8_t vehicle_id) 
{
    timeseries_vehicles[vehicle_id] = map<string, shared_ptr<TimeSeries>>();

    timeseries_vehicles[vehicle_id]["pose_x"] = make_shared<TimeSeries>(
        "Position X", "%6.2f", "m");

    timeseries_vehicles[vehicle_id]["pose_y"] = make_shared<TimeSeries>(
        "Position Y", "%6.2f", "m");

    timeseries_vehicles[vehicle_id]["pose_yaw"] = make_shared<TimeSeries>(
        "Yaw", "%6.3f", "rad");

    timeseries_vehicles[vehicle_id]["ips_x"] = make_shared<TimeSeries>(
        "IPS Position X", "%6.2f", "m");

    timeseries_vehicles[vehicle_id]["ips_y"] = make_shared<TimeSeries>(
        "IPS Position Y", "%6.2f", "m");

    timeseries_vehicles[vehicle_id]["ips_yaw"] = make_shared<TimeSeries>(
        "IPS Yaw", "%6.3f", "rad");

    timeseries_vehicles[vehicle_id]["odometer_distance"] = make_shared<TimeSeries>(
        "Odometer Distance", "%7.2f", "m");

    timeseries_vehicles[vehicle_id]["imu_acceleration_forward"] = make_shared<TimeSeries>(
        "Acceleration Forward", "%4.1f", "m/s^2");

    timeseries_vehicles[vehicle_id]["imu_acceleration_left"] = make_shared<TimeSeries>(
        "Acceleration Left", "%4.1f", "m/s^2");

    timeseries_vehicles[vehicle_id]["speed"] = make_shared<TimeSeries>(
        "Speed", "%5.2f", "m/s");

    timeseries_vehicles[vehicle_id]["battery_voltage"] = make_shared<TimeSeries>(
        "Battery Voltage", "%5.2f", "V");

    timeseries_vehicles[vehicle_id]["motor_current"] = make_shared<TimeSeries>(
        "Motor Current", "%5.2f", "A");

    timeseries_vehicles[vehicle_id]["clock_delta"] = make_shared<TimeSeries>(
        "Clock Delta", "%5.1f", "ms");
}


void TimeSeriesAggregator::handle_new_vehicleState_samples(dds::sub::LoanedSamples<VehicleState>& samples)
{
    std::lock_guard<std::mutex> lock(_mutex); 
    const uint64_t now = clock_gettime_nanoseconds();
    for(auto sample : samples)
    {
        if(sample.info().valid())
        {
            VehicleState state = sample.data();
            if(timeseries_vehicles.count(state.vehicle_id()) == 0)
            {
                create_vehicle_timeseries(state.vehicle_id());
            }

            timeseries_vehicles[state.vehicle_id()]["pose_x"]                   ->push_sample(now, state.pose().x());
            timeseries_vehicles[state.vehicle_id()]["pose_y"]                   ->push_sample(now, state.pose().y());
            timeseries_vehicles[state.vehicle_id()]["pose_yaw"]                 ->push_sample(now, state.pose().yaw());
            timeseries_vehicles[state.vehicle_id()]["odometer_distance"]        ->push_sample(now, state.odometer_distance());
            timeseries_vehicles[state.vehicle_id()]["imu_acceleration_forward"] ->push_sample(now, state.imu_acceleration_forward());
            timeseries_vehicles[state.vehicle_id()]["imu_acceleration_left"]    ->push_sample(now, state.imu_acceleration_left());
            timeseries_vehicles[state.vehicle_id()]["speed"]                    ->push_sample(now, state.speed());
            timeseries_vehicles[state.vehicle_id()]["battery_voltage"]          ->push_sample(now, state.battery_voltage());
            timeseries_vehicles[state.vehicle_id()]["motor_current"]            ->push_sample(now, state.motor_current());

            timeseries_vehicles[state.vehicle_id()]["clock_delta"]              ->push_sample(now, double(now - state.header().create_stamp().nanoseconds())/1e6 );

        }
    }
}


void TimeSeriesAggregator::handle_new_vehicleObservation_samples(dds::sub::LoanedSamples<VehicleObservation>& samples)
{
    std::lock_guard<std::mutex> lock(_mutex); 
    const uint64_t now = clock_gettime_nanoseconds();
    for(auto sample : samples)
    {
        if(sample.info().valid())
        {
            VehicleObservation state = sample.data();
            if(timeseries_vehicles.count(state.vehicle_id()) == 0)
            {
                create_vehicle_timeseries(state.vehicle_id());
            }

            timeseries_vehicles[state.vehicle_id()]["ips_x"]  ->push_sample(now, state.pose().x());
            timeseries_vehicles[state.vehicle_id()]["ips_y"]  ->push_sample(now, state.pose().y());
            timeseries_vehicles[state.vehicle_id()]["ips_yaw"]->push_sample(now, state.pose().yaw());

        }
    }
}




VehicleData TimeSeriesAggregator::get_vehicle_data() {
    std::lock_guard<std::mutex> lock(_mutex); 
    return timeseries_vehicles; 
}