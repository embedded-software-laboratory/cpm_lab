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

/**
 * \brief Definition for VehicleData.
 * 
 * Outer Map: Vehicle ID (uint8_t) -> Inner Map
 * 
 * InnerMap: Identifier (e.g. "ips_x", "speed", ...) -> TimeSeries data for that identifier
 * \ingroup lcc
 */
using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;

/**
 * \brief Definition for VehicleTrajectories.
 * 
 * Vehicle ID (uint8_t) -> VehicleCommandTrajectory data for that vehicle
 * \ingroup lcc
 */
using VehicleTrajectories = map<uint8_t, VehicleCommandTrajectory >;

/**
 * \brief Definition for VehiclePathTracking.
 * 
 * Vehicle ID (uint8_t) -> VehicleCommandPathTracking data for that vehicle
 * \ingroup lcc
 */
using VehiclePathTracking = map<uint8_t, VehicleCommandPathTracking >;

/**
 * \class TimeSeriesAggregator
 * \brief Keeps received data from vehicles and vehicle trajectories in map with custom data structure that regards multiple messages + timestamps
 * \ingroup lcc
*/
class TimeSeriesAggregator
{
    //! Includes all current received vehicle data (pose, speed, battery level...), mapped to a vehicle ID
    VehicleData timeseries_vehicles;

    /**
     * \brief Creates entry for timeseries_vehicles for a vehicle, vehicle ID -> map of IDs (like speed) -> TimeSeries values. 
     * Initializes the second map by constructing the time series entries, e.g. for speed, battery level...
     * \param vehicle_id The vehicle ID to create the entry for
     */
    void create_vehicle_timeseries(uint8_t vehicle_id);

    /**
     * \brief Takes samples by vehicle_state_reader and stores them in timeseries_vehicles.
     * Also checks for deviation regarding the expected update frequency of the received entries.
     * Deviations for entries where no update is performed are detected outside this class,
     * when in the UI the currenty newest sample in timeseries_vehicles is determined to be out of date.
     * \param samples The newly received VehicleState samples
     */
    void handle_new_vehicleState_samples(std::vector<VehicleState>& samples);

    /**
     * \brief Takes samples by vehicle_observation_reader and stores them in timeseries_vehicles.
     * Also checks for deviation regarding the expected update frequency of the received entries.
     * \param samples The newly received VehicleObservation samples
     */
    void handle_new_vehicleObservation_samples(std::vector<VehicleObservation>& samples);

    //! Async. reader to receive vehicle state data from the vehicles and store them for later access in the LCC
    shared_ptr<cpm::AsyncReader<VehicleState>> vehicle_state_reader;
    //! Async. reader to receive vehicle observation data from the IPS and store them for later access in the LCC
    shared_ptr<cpm::AsyncReader<VehicleObservation>> vehicle_observation_reader;
    //! Reader to allow for simple access to newest already valid vehicle trajectories in get_vehicle_trajectory_commands
    shared_ptr<cpm::MultiVehicleReader<VehicleCommandTrajectory>> vehicle_commandTrajectory_reader;
    //! Reader to allow for simple access to newest already valid vehicle path trackings in get_vehicle_path_tracking_commands
    shared_ptr<cpm::MultiVehicleReader<VehicleCommandPathTracking>> vehicle_commandPathTracking_reader;
    //! Vector of vehicle IDs to listen to (every other trajectory msg gets ignored) - Reason: Compatible to MultiVehicleReader. Alternative: MultiVehicleReader that is flexible regarding the vehicle IDs.
    std::vector<uint8_t> vehicle_ids;

    //! For handling new states, resetting all data and getting the vehicle data
    std::mutex _mutex;

    //Expected update frequency and structures to detect changes in update frequency
    //! Expected update frequency
    const uint64_t expected_period_nanoseconds = 20000000ull; // 50 Hz
    //! Allowed deviation from update frequency
    const uint64_t allowed_deviation = expected_period_nanoseconds / 10;
    //! Map vehicle_id -> timestamp of last received vehicle state message
    std::unordered_map<uint8_t, uint64_t> last_vehicle_state_time;
    //! Map vehicle_id -> timestamp of last received vehicle observation message
    std::unordered_map<uint8_t, uint64_t> last_vehicle_observation_time;
    /**
     * \brief Checks if the sample deviates from the expected interval, 
     * resets in that case to prevent spamming in case of periodical checking without value changes
     * \param t_now Current time
     * \param entry Sample with some receive time
     * \param allowed_diff Allowed diff betwwen current time and receive time
     */
    void check_for_deviation(uint64_t t_now, std::unordered_map<uint8_t, uint64_t>::iterator entry, uint64_t allowed_diff);

public:
    /**
     * \brief Constructor
     * \param max_vehicle_id The aggregator does not listen to IDs above this value; must be set for setting listener properly (storage etc)
     */
    TimeSeriesAggregator(uint8_t max_vehicle_id);

    /**
     * \brief Get current received vehicle data
     */
    VehicleData get_vehicle_data();

    /**
     * \brief Get newest received vehicle trajectories that are already valid (using MultiVehicleReader)
     */
    VehicleTrajectories get_vehicle_trajectory_commands();

    /**
     * \brief Get newest received vehicle path trackings that are already valid (using MultiVehicleReader)
     */
    VehiclePathTracking get_vehicle_path_tracking_commands();

    /**
     * \brief Reset the data structures if desired by the user (e.g. bc the simulation was stopped -> reset vehicle data)
     * Also resets trajectory and path reader
     */
    void reset_all_data();
};