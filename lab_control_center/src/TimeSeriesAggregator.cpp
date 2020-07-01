// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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

    //Set vehicle IDs to listen to in the aggregator - 30 is chosen rather arbitrarily, 20 vehicles are planned atm - change if you need higher values as well
    for (uint8_t i = 1; i < 30; ++i)
    {
        vehicle_ids.push_back(i);
    }
    vehicle_commandTrajectory_reader = make_shared<cpm::MultiVehicleReader<VehicleCommandTrajectory>>(
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory"),
        vehicle_ids
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
    
    timeseries_vehicles[vehicle_id]["battery_level"] = make_shared<TimeSeries>(
        "Battery Level", "%3.0f", "%");

    timeseries_vehicles[vehicle_id]["motor_current"] = make_shared<TimeSeries>(
        "Motor Current", "%5.2f", "A");

    timeseries_vehicles[vehicle_id]["clock_delta"] = make_shared<TimeSeries>(
        "Clock Delta", "%5.1f", "ms");
}


static inline double voltage_to_percent(const double& v)
{
    // approximate discharge curve with three linear segments,
    // see tools/linear_discharge.m
    if (v >= 7.55)
    {
        return std::min({72.83 * (v-7.55) + 52.66, 100.0});
    }
    else if (v >= 7.22)
    {
        return (143.45 * (v-7.22) +  5.33);
    }
    else
    {
        return std::max({6.49 * (v-6.4 ), 0.0});
    }
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
            timeseries_vehicles[state.vehicle_id()]["battery_level"]            ->push_sample(now, voltage_to_percent(state.battery_voltage()));
            timeseries_vehicles[state.vehicle_id()]["motor_current"]            ->push_sample(now, state.motor_current());
            timeseries_vehicles[state.vehicle_id()]["clock_delta"]              ->push_sample(now, double(int64_t(now)- int64_t(state.header().create_stamp().nanoseconds()))/1e6 );
        }
    }
}


void TimeSeriesAggregator::handle_new_vehicleObservation_samples(
    dds::sub::LoanedSamples<VehicleObservation>& samples
)
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

VehicleTrajectories TimeSeriesAggregator::get_vehicle_trajectory_commands() {
    VehicleTrajectories trajectory_sample;
    std::map<uint8_t, uint64_t> trajectory_sample_age;
    vehicle_commandTrajectory_reader->get_samples(cpm::get_time_ns(), trajectory_sample, trajectory_sample_age);
    return trajectory_sample;
}

void TimeSeriesAggregator::reset_all_data()
{
    std::lock_guard<std::mutex> lock(_mutex);
    timeseries_vehicles.clear();
    vehicle_commandTrajectory_reader = make_shared<cpm::MultiVehicleReader<VehicleCommandTrajectory>>(
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory"),
        vehicle_ids
    );
}