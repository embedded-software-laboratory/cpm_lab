#pragma once
#include "casadi_mpc_fn.h"
#include <map>
#include <string>
#include <vector>



/* The MPC prediction and the gradient descent 
 * optimization are developed in Matlab with CasADi.
 * Here we just call the resulting generated code.
 * See matlab_scripts/vehicle_dynamics_identification_and_mpc/MpcController.m
 */



class MpcController
{

    std::map< std::string, std::vector<casadi_real> > casadi_vars;

    std::vector<casadi_real*> casadi_arguments;
    std::vector<casadi_real*> casadi_results;


public:

	const uint64_t prediction_timestep_nanoseconds = 50000000ull;
	const size_t prediction_steps = 8;
	const size_t control_steps = 4;

    MpcController();
    
};