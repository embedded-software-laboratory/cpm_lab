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


// CPM Libs
#include "cpm/Logging.hpp"
#include "cpm/get_topic.hpp"
#include "LaneGraphTrajectory.hpp"
#include "cpm/Writer.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/get_topic.hpp"

// DDS files
#include <dds/pub/ddspub.hpp>                   //rti folder
#include <dds/sub/ddssub.hpp>                   //rti folder
#include <dds/domain/DomainParticipant.hpp>         // Required for communication with middleware

class InterVehicleComms
{
    cpm::Writer<LaneGraphTrajectory> trajectory_writer;
    cpm::MultiVehicleReader<LaneGraphTrajectory> trajectory_reader;

    cpm::Writer<PlanningConfirmation> confirmation_writer;
    cpm::MultiVehicleReader<PlanningConfirmation> confirmation_reader;

    // Currently fixed; needs changing
    // Also we only need the vehicle_ids on our layer of comms graph
    std::vector<uint8_t> vehicle_ids;

    unsigned int attempt; // Counts up with each sent message per timestep
    uint64_t t_now;
    // Public
public:
    InterVehicleComms(uint8_t other_vehicle_ids);
    void set_timestep(uint64_t nanos);
    std::map<uint8_t, LaneGraphTrajectory> get_plans();

    void change_plan(LaneGraphTrajectory plan);

    bool is_timestep_resolved(); // This method should block until all okay/not-okay msg received
}
