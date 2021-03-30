//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Mar-2021 12:18:40
//

// Include Files
#include "planTrajectory.h"
#include "PlanRRTPath.h"
#include "pathToTrajectory.h"
#include "planTrajectory_data.h"
#include "planTrajectory_initialize.h"
#include "rt_nonfinite.h"
#include "setCostmap.h"
#include <string.h>

// Function Definitions

//
// Arguments    : const double vehicleIdList_data[]
//                const int vehicleIdList_size[2]
//                const mgen::struct0_T vehiclePoses_data[]
//                const int vehiclePoses_size[2]
//                const mgen::Pose2D *goalPose
//                unsigned char egoVehicleId
//                double speed
//                coder::array<mgen::struct1_T, 1U> *trajectory_points
//                boolean_T *isPathValid
// Return Type  : void
//
namespace mgen
{
  void planTrajectory(const double [], const int vehicleIdList_size[2], const
                      struct0_T vehiclePoses_data[], const int [2], const Pose2D
                      *goalPose, unsigned char egoVehicleId, double speed, coder::
                      array<struct1_T, 1U> &trajectory_points, boolean_T
                      *isPathValid)
  {
    int egoVehicleIndex;
    int nVehicles;
    boolean_T exitg1;
    static c_driving_internal_costmap_Vehi costmap;
    driving_Path refPath;
    if (!isInitialized_planTrajectory) {
      planTrajectory_initialize();
    }

    egoVehicleIndex = -1;
    nVehicles = 0;
    exitg1 = false;
    while ((!exitg1) && (nVehicles <= vehicleIdList_size[1] - 1)) {
      if (vehiclePoses_data[nVehicles].vehicle_id == egoVehicleId) {
        egoVehicleIndex = nVehicles;
        exitg1 = true;
      } else {
        nVehicles++;
      }
    }

    setCostmap(vehiclePoses_data, egoVehicleId, &costmap);
    PlanRRTPath(vehiclePoses_data[egoVehicleIndex].pose, *goalPose, &costmap,
                &refPath, isPathValid);
    if (*isPathValid) {
      pathToTrajectory(&refPath, speed, trajectory_points);
    } else {
      trajectory_points.set_size(1);
      trajectory_points[0].t = 0UL;
      trajectory_points[0].px = 0.0;
      trajectory_points[0].py = 0.0;
      trajectory_points[0].vx = 0.0;
      trajectory_points[0].vy = 0.0;
    }
  }
}

//
// File trailer for planTrajectory.cpp
//
// [EOF]
//
