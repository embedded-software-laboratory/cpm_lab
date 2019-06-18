#pragma once
#include "casadi_mpc_fn.h"
#include <map>
#include <string>
#include <vector>
#include "VehicleState.hpp"
#include "VehicleCommandTrajectory.hpp"



/* The MPC prediction and the gradient descent 
 * optimization are developed in Matlab with CasADi.
 * Here we just call the resulting generated code.
 * See matlab_scripts/vehicle_dynamics_identification_and_mpc/MpcController.m
 */

#define MPC_DELAY_COMPENSATION_STEPS (4)


class MpcController
{

    std::map< std::string, std::vector<casadi_real> > casadi_vars;

    std::vector<casadi_real*> casadi_arguments;
    std::vector<casadi_real*> casadi_results;


    const uint64_t prediction_timestep_nanoseconds = 50000000ull;
    const size_t prediction_steps = 8;
    const size_t control_steps = 4;


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


public:


    MpcController();



    void update(
        uint64_t t_now, 
        const VehicleState &vehicleState,                  
        const std::map<uint64_t, TrajectoryPoint> &trajectory_points,
        double &motor_throttle, 
        double &steering_servo
    );
    
};