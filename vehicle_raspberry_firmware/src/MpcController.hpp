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
    MpcController();
    
};