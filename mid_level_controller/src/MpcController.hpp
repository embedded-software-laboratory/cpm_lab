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
#include "casadi_mpc_fn.h"
#include <map>
#include <string>
#include <array>
#include <vector>
#include "VehicleModel.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleState.hpp"
#include "Visualization.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Writer.hpp"



/* The MPC prediction and the gradient descent 
 * optimization are developed in Matlab with CasADi.
 * Here we just call the resulting generated code.
 * See tools/vehicle_dynamics_identification_and_mpc/MpcController.m
 */

#define MPC_DELAY_COMPENSATION_STEPS (3)


class MpcController
{
    cpm::Writer<Visualization> writer_Visualization;
    uint8_t vehicle_id;

    std::map< std::string, std::vector<casadi_real> > casadi_vars;
    std::map< std::string, std::array<casadi_int, 2> > casadi_vars_size;


    std::vector<casadi_real*> casadi_arguments;
    std::vector<casadi_real*> casadi_results;


    const size_t MPC_prediction_steps = 6;
    const size_t MPC_control_steps = 3;
    const double dt_control_loop = 0.02; // the period in which update() is called
    const double dt_MPC = 0.05; // the MPC prediction time step


    // TODO load parameters via DDS parameters
    std::vector<double> dynamics_parameters = { 1.004582, -0.142938, 0.195236, 3.560576, -2.190728, -9.726828, 2.515565, 1.321199, 0.032208, -0.012863 };

    double battery_voltage_lowpass_filtered = 8;


    // holds the N most recent outputs/commands. oldest first, newest last
    double motor_output_history[MPC_DELAY_COMPENSATION_STEPS];
    double steering_output_history[MPC_DELAY_COMPENSATION_STEPS];


    // Take the current state measurement and predict it into the future a few steps.
    // This is necessary to compensate the delay of the inputs.
    VehicleState delay_compensation_prediction(
        const VehicleState &vehicleState
    );


    // Interpolates the reference trajectory on the MPC time grid.
    // Returns false if the reference trajectory is not defined for the 
    // MPC prediction time interval, or is impossible to follow.
    bool interpolate_reference_trajectory(
        uint64_t t_now, 
        const VehicleCommandTrajectory &commandTrajectory,
        std::vector<double> &out_mpc_reference_trajectory_x,
        std::vector<double> &out_mpc_reference_trajectory_y
    );



    void optimize_control_inputs(
        const VehicleState &vehicleState_predicted_start,
        const std::vector<double> &mpc_reference_trajectory_x,
        const std::vector<double> &mpc_reference_trajectory_y,
        double &out_motor_throttle, 
        double &out_steering_servo
    );

    void reset_optimizer();

    std::function<void(double&, double&)> stop_vehicle;


public:

    MpcController(uint8_t _vehicle_id, std::function<void(double&, double&)> _stop_vehicle);

    void update(
        uint64_t t_now, 
        const VehicleState &vehicleState,                  
        const VehicleCommandTrajectory &commandTrajectory,
        double &out_motor_throttle, 
        double &out_steering_servo
    );
    
};