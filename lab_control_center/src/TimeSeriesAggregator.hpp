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

#pragma once
#include "defaults.hpp"

#include "VehicleState.hpp"
#include "VehicleObservation.hpp"
#include "TimeSeries.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleCommandPathTracking.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/Logging.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/get_time_ns.hpp"

#include <mutex>
#include <unordered_map>


using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;
using VehicleTrajectories = map<uint8_t, VehicleCommandTrajectory >;
using VehiclePathTracking = map<uint8_t, VehicleCommandPathTracking >;

/**
 * \class TimeSeriesAggregator
 * \brief Keeps received data from vehicles and vehicle trajectories in map with custom data structure that regards multiple messages + timestamps
 *
*/
class TimeSeriesAggregator
{
    VehicleData timeseries_vehicles;

    void create_vehicle_timeseries(uint8_t vehicle_id);
    void handle_new_vehicleState_samples(std::vector<VehicleState>& samples);
    void handle_new_vehicleObservation_samples(std::vector<VehicleObservation>& samples);

    shared_ptr<cpm::AsyncReader<VehicleState>> vehicle_state_reader;
    shared_ptr<cpm::AsyncReader<VehicleObservation>> vehicle_observation_reader;
    shared_ptr<cpm::MultiVehicleReader<VehicleCommandTrajectory>> vehicle_commandTrajectory_reader;
    shared_ptr<cpm::MultiVehicleReader<VehicleCommandPathTracking>> vehicle_commandPathTracking_reader;
    std::vector<uint8_t> vehicle_ids; //Vector of vehicle IDs to listen to (every other trajectory msg gets ignored) - Reason: Compatible to MultiVehicleReader
    //Alternative: MultiVehicleReader that is flexible regarding the vehicle IDs

    std::mutex _mutex;

    //Expected update frequency and structures to detect changes in update frequency
    const uint64_t expected_period_nanoseconds = 20000000ull; // 50 Hz
    const uint64_t allowed_deviation = expected_period_nanoseconds / 10;
    std::unordered_map<uint8_t, uint64_t> last_vehicle_state_time;
    std::unordered_map<uint8_t, uint64_t> last_vehicle_observation_time;
    //Checks if the sample deviates from the expected interval, resets in that case to prevent spamming in case of periodical checking without value changes
    void check_for_deviation(uint64_t t_now, std::unordered_map<uint8_t, uint64_t>::iterator entry, uint64_t allowed_diff);

    //! Max. allowed data age before data is totally ignored or reset if possible - currently at 1 second
    const uint64_t max_allowed_age = 1e9;

public:
    /**
     * \brief Constructor
     * \param max_vehicle_id The aggregator does not listen to IDs above this value; must be set for setting listener properly (storage etc)
     */
    TimeSeriesAggregator(uint8_t max_vehicle_id);
    VehicleData get_vehicle_data();
    VehicleTrajectories get_vehicle_trajectory_commands();
    VehiclePathTracking get_vehicle_path_tracking_commands();
    void reset_all_data(); //Reset the data structures if desired by the user (e.g. bc the simulation was stopped)
};