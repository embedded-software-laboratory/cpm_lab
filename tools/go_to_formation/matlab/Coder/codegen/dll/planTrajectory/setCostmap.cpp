//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: setCostmap.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
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
// Arguments    : const mgen::struct0_T vehiclePoses[20]
//                double egoVehicleId
//                mgen::c_driving_internal_costmap_Vehi *iobj_0
// Return Type  : mgen::c_driving_internal_costmap_Vehi *
//
namespace mgen
{
  c_driving_internal_costmap_Vehi *setCostmap(const struct0_T vehiclePoses[20],
    double egoVehicleId, c_driving_internal_costmap_Vehi *iobj_0)
  {
    c_driving_internal_costmap_Vehi *costmap;
    boolean_T isVehicleDeployed;
    double corners_tmp_tmp;
    double b_corners_tmp_tmp;
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
    double corners[8];
    double b_vehiclePoses[2];
    costmap = iobj_0->init();

    //     %% Calculate corners of vehicles from pose and set to costmap
    isVehicleDeployed = true;
    for (int nVehicles = 0; nVehicles < 20; nVehicles++) {
      //  All vehicles not actually deployed are ignored.
      //  Passing of all theoretically deployable vehicles necessary for
      //  automatic code generation from matlab.
      if ((vehiclePoses[nVehicles].pose.x == 0.0) && (vehiclePoses[nVehicles].
           pose.y == 0.0) && (vehiclePoses[nVehicles].pose.yaw == 0.0)) {
        isVehicleDeployed = false;
      }

      if ((!(vehiclePoses[nVehicles].vehicle_id == egoVehicleId)) &&
          isVehicleDeployed) {
        double corners_tmp;
        corners_tmp_tmp = vehiclePoses[nVehicles].pose.yaw;
        b_sind(&corners_tmp_tmp);
        b_corners_tmp_tmp = vehiclePoses[nVehicles].pose.yaw;
        b_cosd(&b_corners_tmp_tmp);
        d = vehiclePoses[nVehicles].pose.yaw;
        b_cosd(&d);
        d1 = vehiclePoses[nVehicles].pose.yaw;
        b_sind(&d1);
        d2 = vehiclePoses[nVehicles].pose.yaw;
        b_cosd(&d2);
        d3 = vehiclePoses[nVehicles].pose.yaw;
        b_sind(&d3);
        d4 = vehiclePoses[nVehicles].pose.yaw;
        b_cosd(&d4);
        d5 = vehiclePoses[nVehicles].pose.yaw;
        b_sind(&d5);
        corners_tmp = vehiclePoses[nVehicles].pose.y + corners_tmp_tmp * 0.11;
        d6 = vehiclePoses[nVehicles].pose.yaw;
        b_sind(&d6);
        d7 = vehiclePoses[nVehicles].pose.yaw;
        b_cosd(&d7);
        d8 = vehiclePoses[nVehicles].pose.yaw;
        b_sind(&d8);
        d9 = vehiclePoses[nVehicles].pose.yaw;
        b_cosd(&d9);
        corners[0] = (vehiclePoses[nVehicles].pose.x + b_corners_tmp_tmp * 0.11)
          + corners_tmp_tmp * 0.0535;
        corners[4] = corners_tmp - b_corners_tmp_tmp * 0.0535;
        corners[1] = (vehiclePoses[nVehicles].pose.x + d * 0.11) - d1 * 0.0535;
        corners[5] = corners_tmp + b_corners_tmp_tmp * 0.0535;
        corners[2] = (vehiclePoses[nVehicles].pose.x - d2 * 0.11) - d3 * 0.0535;
        corners[6] = (vehiclePoses[nVehicles].pose.y - d6 * 0.11) + d7 * 0.0535;
        corners[3] = (vehiclePoses[nVehicles].pose.x - d4 * 0.11) + d5 * 0.0535;
        corners[7] = (vehiclePoses[nVehicles].pose.y - d8 * 0.11) - d9 * 0.0535;
        costmap->setCosts(corners);
        b_vehiclePoses[0] = vehiclePoses[nVehicles].pose.x;
        b_vehiclePoses[1] = vehiclePoses[nVehicles].pose.y;
        costmap->VehicleCostmapImpl_setCosts(b_vehiclePoses);
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
