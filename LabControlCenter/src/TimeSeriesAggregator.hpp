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
#include <dds/sub/ddssub.hpp>
#include "VehicleState.hpp"
#include "VehicleObservation.hpp"
#include "TimeSeries.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "cpm/AsyncReader.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include <mutex>


using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;
using VehicleTrajectories = map<uint8_t, VehicleCommandTrajectory >;

/**
 * \class TimeSeriesAggregator
 * \brief Keeps received data from vehicles and vehicle trajectories in map with custom data structure that regards multiple messages + timestamps
 *
*/

class TimeSeriesAggregator
{
    VehicleData timeseries_vehicles;

    void create_vehicle_timeseries(uint8_t vehicle_id);
    void handle_new_vehicleState_samples(dds::sub::LoanedSamples<VehicleState>& samples);
    void handle_new_vehicleObservation_samples(dds::sub::LoanedSamples<VehicleObservation>& samples);

    shared_ptr<cpm::AsyncReader<VehicleState>> vehicle_state_reader;
    shared_ptr<cpm::AsyncReader<VehicleObservation>> vehicle_observation_reader;
    shared_ptr<cpm::MultiVehicleReader<VehicleCommandTrajectory>> vehicle_commandTrajectory_reader;
    std::vector<uint8_t> vehicle_ids; //Vector of vehicle IDs to listen to (every other trajectory msg gets ignored) - Reason: Compatible to MultiVehicleReader
    //Alternative: MultiVehicleReader that is flexible regarding the vehicle IDs

    std::mutex _mutex;

public:
    TimeSeriesAggregator();
    VehicleData get_vehicle_data();
    VehicleTrajectories get_vehicle_trajectory_commands();
    void reset_all_data(); //Reset the data structures if desired by the user (e.g. bc the simulation was stopped)
};