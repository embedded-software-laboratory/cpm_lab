//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: setCostmap.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Mar-2021 23:01:38
//
#ifndef SETCOSTMAP_H
#define SETCOSTMAP_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "omp.h"
#include "planTrajectory_types.h"
#define MAX_THREADS                    omp_get_max_threads()

// Function Declarations
namespace mgen
{
  extern c_driving_internal_costmap_Vehi *setCostmap(const struct0_T
    vehiclePoses_data[], unsigned char egoVehicleId,
    c_driving_internal_costmap_Vehi *iobj_0);
}

#endif

//
// File trailer for setCostmap.h
//
// [EOF]
//
