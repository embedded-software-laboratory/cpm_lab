//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planTrajectory.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jan-2021 13:31:05
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
// Arguments    : void
// Return Type  : void
//
namespace mgen
{
  DubinsBuiltins::~DubinsBuiltins()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  DubinsBuiltins::DubinsBuiltins()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  Pose2D::Pose2D()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  Pose2D::~Pose2D()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  angleUtilities::~angleUtilities()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  angleUtilities::angleUtilities()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  c_driving_internal_costmap_Vehi::~c_driving_internal_costmap_Vehi()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  c_driving_internal_costmap_Vehi::c_driving_internal_costmap_Vehi()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  c_driving_internal_planning_Dub::c_driving_internal_planning_Dub()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  c_driving_internal_planning_Dub::~c_driving_internal_planning_Dub()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  c_matlabshared_autonomous_core_::~c_matlabshared_autonomous_core_()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  c_matlabshared_autonomous_core_::c_matlabshared_autonomous_core_()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  c_matlabshared_planning_interna::c_matlabshared_planning_interna()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  c_matlabshared_planning_interna::~c_matlabshared_planning_interna()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  coder_internal_stack::~coder_internal_stack()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  coder_internal_stack::coder_internal_stack()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  d_driving_internal_planning_Dub::d_driving_internal_planning_Dub()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  d_driving_internal_planning_Dub::~d_driving_internal_planning_Dub()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  d_matlabshared_autonomous_core_::~d_matlabshared_autonomous_core_()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  d_matlabshared_autonomous_core_::d_matlabshared_autonomous_core_()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  d_matlabshared_planning_interna::~d_matlabshared_planning_interna()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  d_matlabshared_planning_interna::d_matlabshared_planning_interna()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  driving_Path::~driving_Path()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  driving_Path::driving_Path()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  e_driving_internal_planning_Dub::e_driving_internal_planning_Dub()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  e_driving_internal_planning_Dub::~e_driving_internal_planning_Dub()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  e_matlabshared_planning_interna::~e_matlabshared_planning_interna()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  e_matlabshared_planning_interna::e_matlabshared_planning_interna()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  f_matlabshared_planning_interna::~f_matlabshared_planning_interna()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  f_matlabshared_planning_interna::f_matlabshared_planning_interna()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  g_matlabshared_planning_interna::~g_matlabshared_planning_interna()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  g_matlabshared_planning_interna::g_matlabshared_planning_interna()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  pathPlannerRRT::~pathPlannerRRT()
  {
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
  pathPlannerRRT::pathPlannerRRT()
  {
  }

  //
  // Arguments    : const double vehicleIdList[20]
  //                const struct0_T vehiclePoses[20]
  //                const Pose2D *goalPose
  //                double egoVehicleId
  //                double speed
  //                coder::array<struct1_T, 1U> *trajectory_points
  //                boolean_T *isPathValid
  // Return Type  : void
  //
  void planTrajectory(const double [20], const struct0_T vehiclePoses[20], const
                      Pose2D *goalPose, double egoVehicleId, double speed, coder::
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
    while ((!exitg1) && (nVehicles < 20)) {
      if (vehiclePoses[nVehicles].vehicle_id == egoVehicleId) {
        egoVehicleIndex = nVehicles;
        exitg1 = true;
      } else {
        nVehicles++;
      }
    }

    setCostmap(vehiclePoses, egoVehicleId, &costmap);
    PlanRRTPath(vehiclePoses[egoVehicleIndex].pose, *goalPose, &costmap,
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
