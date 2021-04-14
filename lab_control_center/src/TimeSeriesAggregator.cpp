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

/**
 * \file TimeSeriesAggregator.cpp
 * \ingroup lcc
 */

TimeSeriesAggregator::TimeSeriesAggregator(uint8_t max_vehicle_id)
{
    vehicle_state_reader = make_shared<cpm::AsyncReader<VehicleState>>(
        [this](std::vector<VehicleState>& samples){
            handle_new_vehicleState_samples(samples);
        },
        "vehicleState"
    );


    vehicle_observation_reader = make_shared<cpm::AsyncReader<VehicleObservation>>(
        [this](std::vector<VehicleObservation>& samples){
            handle_new_vehicleObservation_samples(samples);
        },
        "vehicleObservation"
    );

    //Set vehicle IDs to listen to in the aggregator
    for (uint8_t i = 1; i < max_vehicle_id; ++i)
    {
        vehicle_ids.push_back(i);
    }
    vehicle_commandTrajectory_reader = make_shared<cpm::MultiVehicleReader<VehicleCommandTrajectory>>(
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory"),
        vehicle_ids
    );
    vehicle_commandPathTracking_reader = make_shared<cpm::MultiVehicleReader<VehicleCommandPathTracking>>(
        cpm::get_topic<VehicleCommandPathTracking>("vehicleCommandPathTracking"),
        vehicle_ids
    );
}


void TimeSeriesAggregator::create_vehicle_timeseries(uint8_t vehicle_id) 
{
    timeseries_vehicles[vehicle_id] = map<string, shared_ptr<TimeSeries>>();

    timeseries_vehicles[vehicle_id]["reference_deviation"] = make_shared<TimeSeries>(
        "Reference Deviation", "%6.2f", "m");

    timeseries_vehicles[vehicle_id]["pose_x"] = make_shared<TimeSeries>(
        "Position X", "%6.2f", "m");

    timeseries_vehicles[vehicle_id]["pose_y"] = make_shared<TimeSeries>(
        "Position Y", "%6.2f", "m");

    timeseries_vehicles[vehicle_id]["pose_yaw"] = make_shared<TimeSeries>(
        "Yaw", "%6.3f", "rad");

    timeseries_vehicles[vehicle_id]["ips_dt"] = make_shared<TimeSeries>(
        "IPS age", "%3.0f", "ms");

    timeseries_vehicles[vehicle_id]["speed"] = make_shared<TimeSeries>(
        "Speed", "%5.2f", "m/s");
    
    timeseries_vehicles[vehicle_id]["battery_level"] = make_shared<TimeSeries>(
        "Battery Level", "%3.0f", "%");

    timeseries_vehicles[vehicle_id]["clock_delta"] = make_shared<TimeSeries>(
        "Clock Delta", "%5.1f", "ms");

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

    timeseries_vehicles[vehicle_id]["battery_voltage"] = make_shared<TimeSeries>(
        "Battery Voltage", "%5.2f", "V");

    timeseries_vehicles[vehicle_id]["motor_current"] = make_shared<TimeSeries>(
        "Motor Current", "%5.2f", "A");

    timeseries_vehicles[vehicle_id]["is_real"] = make_shared<TimeSeries>(
        "Is Real", "%d", "-");

    //To detect deviations from the required message frequency
    timeseries_vehicles[vehicle_id]["last_msg_state"] = make_shared<TimeSeries>(
    "VehicleState age", "%ull", "ms");
    timeseries_vehicles[vehicle_id]["last_msg_observation"] = make_shared<TimeSeries>(
    "VehicleObservation age", "%ull", "ms");

}

/**
 * \brief return battery level based on voltage. Approximates remaining runtime
          see tools/battery_level/main.m
 * \param v battery voltage
 * \ingroup lcc
 */
static inline double voltage_to_percent(const double& v)
{
    double u1 = 8.17;
    double l1 = 100;
    double u2 = 7.38;
    double l2 = 50;
    double u3 = 7.3;
    double l3 = 12;
    double u4 = 6.3;
    double l4 = 0;

    double battery_level;
    if (v >= u2) {
        battery_level = std::min(100.0, l2 + (l1-l2)/(u1-u2) * (v-u2));
    } else if (v > u3) {
        battery_level = l3 + (l2-l3)/(u2-u3) * (v-u3);
    } else {
        battery_level = std::max(0.0, l4 + (l3-l4)/(u3-u4) * (v-u4));
    }
    return battery_level;
}


void TimeSeriesAggregator::handle_new_vehicleState_samples(std::vector<VehicleState>& samples)
{
    std::lock_guard<std::mutex> lock(_mutex); 
    const uint64_t now = cpm::get_time_ns();
    for(auto& state : samples)
    {
        if(timeseries_vehicles.count(state.vehicle_id()) == 0)
        {
            create_vehicle_timeseries(state.vehicle_id());
        }
        timeseries_vehicles[state.vehicle_id()]["pose_x"]                   ->push_sample(now, state.pose().x());
        timeseries_vehicles[state.vehicle_id()]["pose_y"]                   ->push_sample(now, state.pose().y());
        timeseries_vehicles[state.vehicle_id()]["pose_yaw"]                 ->push_sample(now, state.pose().yaw());
        timeseries_vehicles[state.vehicle_id()]["speed"]                    ->push_sample(now, state.speed());
        timeseries_vehicles[state.vehicle_id()]["battery_level"]            ->push_sample(now, voltage_to_percent(state.battery_voltage()));
        timeseries_vehicles[state.vehicle_id()]["clock_delta"]              ->push_sample(now, double(int64_t(now)- int64_t(state.header().create_stamp().nanoseconds()))/1e6 );
        timeseries_vehicles[state.vehicle_id()]["odometer_distance"]        ->push_sample(now, state.odometer_distance());
        timeseries_vehicles[state.vehicle_id()]["imu_acceleration_forward"] ->push_sample(now, state.imu_acceleration_forward());
        timeseries_vehicles[state.vehicle_id()]["imu_acceleration_left"]    ->push_sample(now, state.imu_acceleration_left());
        timeseries_vehicles[state.vehicle_id()]["battery_voltage"]          ->push_sample(now, state.battery_voltage());
        timeseries_vehicles[state.vehicle_id()]["motor_current"]            ->push_sample(now, state.motor_current());
        timeseries_vehicles[state.vehicle_id()]["is_real"]                  ->push_sample(now, state.is_real());
        // initialize reference deviation, since no reference is available at start 
        timeseries_vehicles[state.vehicle_id()]["reference_deviation"]      ->push_sample(now, 0.0);
        timeseries_vehicles[state.vehicle_id()]["ips_dt"]                   ->push_sample(now, static_cast<double>(1e-6*state.IPS_update_age_nanoseconds()));
        //To detect deviations from the required message frequency
        timeseries_vehicles[state.vehicle_id()]["last_msg_state"]           ->push_sample(now, static_cast<double>(1e-6*now)); //Just remember the latest msg time and calculate diff in the UI

        //Check for deviation from expected update frequency once, reset if deviation was detected
        auto it = last_vehicle_state_time_dev.find(state.vehicle_id());
        if (it != last_vehicle_state_time_dev.end())
        {
            check_for_deviation(now, it, expected_period_nanoseconds + allowed_deviation);
        }

        //Set (first time) or update the value for this ID
        last_vehicle_state_time[state.vehicle_id()] = now;
        last_vehicle_state_time_dev[state.vehicle_id()] = now;
    }
}

void TimeSeriesAggregator::check_for_deviation(uint64_t t_now, std::unordered_map<uint8_t, uint64_t>::iterator entry, uint64_t allowed_diff)
{
    if (entry->second > 0)
    {
        if (t_now - entry->second > allowed_diff)
        {
            cpm::Logging::Instance().write(2, "Vehicle %i deviated from expected vehicle state frequency on LCC side or is offline", static_cast<int>(entry->first));
            entry->second = 0;
        }
        else if (t_now < entry->second)
        {
            //This should never occur, due to the way timestamps are stored, unless the clock values are obtained in a way that negative clock changes of the system change the timestamps
            cpm::Logging::Instance().write(1, "Critical error in TimeSeriesAggregator check, this should never happen; (vehicle id %i)", static_cast<int>(entry->second));
        }
    }
}

void TimeSeriesAggregator::handle_new_vehicleObservation_samples(
    std::vector<VehicleObservation>& samples
)
{
    std::lock_guard<std::mutex> lock(_mutex); 
    const uint64_t now = cpm::get_time_ns();
    for(auto& state : samples)
    {
        if(timeseries_vehicles.count(state.vehicle_id()) == 0)
        {
            create_vehicle_timeseries(state.vehicle_id());
        }
        timeseries_vehicles[state.vehicle_id()]["ips_x"]  ->push_sample(now, state.pose().x());
        timeseries_vehicles[state.vehicle_id()]["ips_y"]  ->push_sample(now, state.pose().y());
        timeseries_vehicles[state.vehicle_id()]["ips_yaw"]->push_sample(now, state.pose().yaw());
        // timeseries to check if any IPS data are available, push any data 
        //timeseries_vehicles[state.vehicle_id()]["ips"]    ->push_sample(now, true);
        //To detect deviations from the required message frequency
        timeseries_vehicles[state.vehicle_id()]["last_msg_observation"] ->push_sample(now, static_cast<double>(1e-6*now)); //Just remember the latest msg time and calculate diff in the UI

        //Check for long intervals without new information - TODO: WHICH VALUE MAKES SENSE HERE?
        auto it = last_vehicle_observation_time.find(state.vehicle_id());
        if (it != last_vehicle_observation_time.end())
        {
            //Currently: Only warn if no new observation sample has been received for over a second - TODO
            check_for_deviation(now, it, expected_period_nanoseconds + allowed_deviation);
        }

        //Set (first time) or update the value for this ID
        last_vehicle_observation_time[state.vehicle_id()] = now;
    }
}

VehicleData TimeSeriesAggregator::get_vehicle_data() {
    std::lock_guard<std::mutex> lock(_mutex); 
    const uint64_t now = cpm::get_time_ns();

    //--------------------------------------------------------------------------- CHECKS ------------------------------------
    //This function is called regularly in the UI, so we make sure that everything is checked regularly just by putting the tests in here as well
    // - Check for deviations in vehicle state msgs
    for (auto it = last_vehicle_state_time.begin(); it != last_vehicle_state_time.end(); /*No ++ because this depends on whether a deletion took place*/)
    {
        //We use another structure for check_for_deviation here, because that function manipulates the entries given the iterator (may set to zero)
        auto it_dev = last_vehicle_state_time_dev.find(it->first);
        if (it_dev != last_vehicle_state_time_dev.end())
        {
            check_for_deviation(now, it_dev, expected_period_nanoseconds + allowed_deviation);
        }

        //Remove entry (also from timeseries) if outdated
        if (now - it->second > max_allowed_age)
        {
            last_vehicle_observation_time.erase(it->first);
            timeseries_vehicles.erase(it->first);
            it = last_vehicle_state_time.erase(it);
        }
        else
        {
            ++it;
        }
    }
    //--------------------------------------------------------------------------- ------- ------------------------------------

    return timeseries_vehicles; 
}

VehicleTrajectories TimeSeriesAggregator::get_vehicle_trajectory_commands() {
    VehicleTrajectories trajectory_sample;
    std::map<uint8_t, uint64_t> trajectory_sample_age;
    vehicle_commandTrajectory_reader->get_samples(cpm::get_time_ns(), trajectory_sample, trajectory_sample_age);

    //Only return data that is not fully outdated
    for(auto it = trajectory_sample.begin(); it != trajectory_sample.end(); /*No ++ because this depends on whether a deletion took place*/)
    {
        if(trajectory_sample_age.at(it->first) > max_allowed_age)
        {
            it = trajectory_sample.erase(it);
        }
        else
        {
            ++it;
        }
    }

    return trajectory_sample;
}

VehiclePathTracking TimeSeriesAggregator::get_vehicle_path_tracking_commands() {
    VehiclePathTracking path_tracking_sample;
    std::map<uint8_t, uint64_t> path_tracking_sample_age;
    vehicle_commandPathTracking_reader->get_samples(cpm::get_time_ns(), path_tracking_sample, path_tracking_sample_age);

    //Only return data that is not fully outdated
    for(auto it = path_tracking_sample.begin(); it != path_tracking_sample.end(); /*No ++ because this depends on whether a deletion took place*/)
    {
        if(path_tracking_sample_age.at(it->first) > max_allowed_age)
        {
            it = path_tracking_sample.erase(it);
        }
        else
        {
            ++it;
        }
    }

    return path_tracking_sample;
}

void TimeSeriesAggregator::reset_all_data()
{
    std::lock_guard<std::mutex> lock(_mutex);
    timeseries_vehicles.clear();
    vehicle_commandTrajectory_reader = make_shared<cpm::MultiVehicleReader<VehicleCommandTrajectory>>(
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory"),
        vehicle_ids
    );
    vehicle_commandPathTracking_reader = make_shared<cpm::MultiVehicleReader<VehicleCommandPathTracking>>(
        cpm::get_topic<VehicleCommandPathTracking>("vehicleCommandPathTracking"),
        vehicle_ids
    );
}