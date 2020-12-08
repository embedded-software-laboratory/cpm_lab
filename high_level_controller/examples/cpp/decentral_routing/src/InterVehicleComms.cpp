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

#include "InterVehicleComms.hpp"

void InterVehicleComms::InterVehicleComms(
            std::vector<uint8_t> other_vehicle_ids
        ){

    vehicle_ids = ids;

    // Create cpm::MultiVehicleReaders
    trajectory_reader(
            cpm::get_topic("laneGraphTrajectory"),
            vehicle_ids
            );
    confirmation_reader(
            cpm::get_topic("planningConfirmation"),
            vehicle_ids
            );

    // Initialize cpm::Writer
    trajectory_writer("laneGraphTrajectory");
}

void InterVehicleComms::set_timestep(uint64_t nanos) {
    attempt = 0;
    t_now = nanos;
}

std::map<uint8_t, LaneGraphTrajectory> InterVehicleComms::get_plans(){
    
}

/*
 * Suggest a new plan, which means previous negotiations failed.
 * Attempt is increased here
 */
void InterVehicleComms::send_plans(LaneGraphTrajectory plan){
    // When should attempt be increased?
    attempt++;
    plan.attempt(attempt);
    writer.write(plan);
}

/*
 * Returns true, if all vehicles have ok'ed each plan.
 */
bool InterVehicleComms::is_timestep_resolved(){
    
}

bool InterVehicleComms::new_msg_received(){

}
