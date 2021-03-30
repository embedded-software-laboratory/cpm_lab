//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: setCostmap.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "setCostmap.h"
#include "VehicleCostmapCodegen.h"
#include "VehicleCostmapImpl.h"
#include "cosd.h"
#include "planTrajectory.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include <string.h>

// Function Definitions

//
// %% Initialize costmap
// Arguments    : const mgen::struct0_T vehiclePoses_data[]
//                unsigned char egoVehicleId
//                mgen::c_driving_internal_costmap_Vehi *iobj_0
// Return Type  : mgen::c_driving_internal_costmap_Vehi *
//
namespace mgen
{
  c_driving_internal_costmap_Vehi *setCostmap(const struct0_T vehiclePoses_data[],
    unsigned char egoVehicleId, c_driving_internal_costmap_Vehi *iobj_0)
  {
    c_driving_internal_costmap_Vehi *costmap;
    boolean_T isVehicleDeployed;
    double d;
    double d1;
    double d2;
    double d3;
    double d4;
    double d5;
    double d6;
    double d7;
    double d8;
    double d9;
    double d10;
    double d11;
    double d12;
    double d13;
    double d14;
    double d15;
    double corners[8];
    double vehiclePoses[2];
    costmap = iobj_0->init();

    //     %% Calculate corners of vehicles from pose and set to costmap
    isVehicleDeployed = true;
    for (int nVehicles = 0; nVehicles < 20; nVehicles++) {
      //  All vehicles not actually deployed are ignored.
      //  Passing of all theoretically deployable vehicles necessary for
      //  automatic code generation from matlab.
      if ((vehiclePoses_data[nVehicles].pose.x == 0.0) &&
          (vehiclePoses_data[nVehicles].pose.y == 0.0) &&
          (vehiclePoses_data[nVehicles].pose.yaw == 0.0)) {
        isVehicleDeployed = false;
      }

      if ((vehiclePoses_data[nVehicles].vehicle_id != egoVehicleId) &&
          isVehicleDeployed) {
        d = vehiclePoses_data[nVehicles].pose.yaw;
        b_cosd(&d);
        d1 = vehiclePoses_data[nVehicles].pose.yaw;
        b_sind(&d1);
        d2 = vehiclePoses_data[nVehicles].pose.yaw;
        b_cosd(&d2);
        d3 = vehiclePoses_data[nVehicles].pose.yaw;
        b_sind(&d3);
        d4 = vehiclePoses_data[nVehicles].pose.yaw;
        b_cosd(&d4);
        d5 = vehiclePoses_data[nVehicles].pose.yaw;
        b_sind(&d5);
        d6 = vehiclePoses_data[nVehicles].pose.yaw;
        b_cosd(&d6);
        d7 = vehiclePoses_data[nVehicles].pose.yaw;
        b_sind(&d7);
        d8 = vehiclePoses_data[nVehicles].pose.yaw;
        b_sind(&d8);
        d9 = vehiclePoses_data[nVehicles].pose.yaw;
        b_cosd(&d9);
        d10 = vehiclePoses_data[nVehicles].pose.yaw;
        b_sind(&d10);
        d11 = vehiclePoses_data[nVehicles].pose.yaw;
        b_cosd(&d11);
        d12 = vehiclePoses_data[nVehicles].pose.yaw;
        b_sind(&d12);
        d13 = vehiclePoses_data[nVehicles].pose.yaw;
        b_cosd(&d13);
        d14 = vehiclePoses_data[nVehicles].pose.yaw;
        b_sind(&d14);
        d15 = vehiclePoses_data[nVehicles].pose.yaw;
        b_cosd(&d15);
        corners[0] = (vehiclePoses_data[nVehicles].pose.x + d * 0.11) + d1 *
          0.0535;
        corners[4] = (vehiclePoses_data[nVehicles].pose.y + d8 * 0.11) - d9 *
          0.0535;
        corners[1] = (vehiclePoses_data[nVehicles].pose.x + d2 * 0.11) - d3 *
          0.0535;
        corners[5] = (vehiclePoses_data[nVehicles].pose.y + d10 * 0.11) + d11 *
          0.0535;
        corners[2] = (vehiclePoses_data[nVehicles].pose.x - d4 * 0.11) - d5 *
          0.0535;
        corners[6] = (vehiclePoses_data[nVehicles].pose.y - d12 * 0.11) + d13 *
          0.0535;
        corners[3] = (vehiclePoses_data[nVehicles].pose.x - d6 * 0.11) + d7 *
          0.0535;
        corners[7] = (vehiclePoses_data[nVehicles].pose.y - d14 * 0.11) - d15 *
          0.0535;
        costmap->setCosts(corners);
        vehiclePoses[0] = vehiclePoses_data[nVehicles].pose.x;
        vehiclePoses[1] = vehiclePoses_data[nVehicles].pose.y;
        costmap->VehicleCostmapImpl_setCosts(vehiclePoses);
      }
    }

    //  Set map limits to costmap
    costmap->setCosts();
    return costmap;
  }
}

//
// File trailer for setCostmap.cpp
//
// [EOF]
//
