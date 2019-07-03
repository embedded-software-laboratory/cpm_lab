#pragma once
#include "casadi_mpc_fn.h"
#include <map>
#include <string>
#include <array>
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
    std::map< std::string, std::array<casadi_int, 2> > casadi_vars_size;


    std::vector<casadi_real*> casadi_arguments;
    std::vector<casadi_real*> casadi_results;


    const size_t MPC_prediction_steps = 8;
    const size_t MPC_control_steps = 4;
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
        const std::map<uint64_t, TrajectoryPoint> &trajectory_points,
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




public:

    MpcController();

    void update(
        uint64_t t_now, 
        const VehicleState &vehicleState,                  
        const std::map<uint64_t, TrajectoryPoint> &trajectory_points,
        double &out_motor_throttle, 
        double &out_steering_servo
    );
    
};