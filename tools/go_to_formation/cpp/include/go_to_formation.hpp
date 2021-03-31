#include "mgen_init.hpp"
#include "planTrajectory.h"
#include "planTrajectory_terminate.h"
#include "rt_nonfinite.h"

#define _USE_MATH_DEFINES

#include "cpm/Logging.hpp"                      //->cpm_lib->include->cpm
#include "cpm/CommandLineReader.hpp"            //->cpm_lib->include->cpm
#include "cpm/init.hpp"                         //->cpm_lib->include->cpm
#include "cpm/MultiVehicleReader.hpp"           //->cpm_lib->include->cpm
#include "cpm/ParticipantSingleton.hpp"         //->cpm_lib->include->cpm
#include "cpm/Timer.hpp"                        //->cpm_lib->include->cpm
#include "cpm/Writer.hpp"
#include "VehicleObservation.hpp" 
#include "VehicleCommandTrajectory.hpp"
#include "Pose2D.hpp"

#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>
#include <map>
#include <cmath>
#include <chrono>
#include <ctime>
#include <string.h>

using std::vector;

int go_to_formation(int argc, char *argv[]);
void set_home_poses(int n_vehicles,  vector<mgen::Pose2D> &home_poses);
void set_goal_poses_from_argc(vector<mgen::Pose2D> &poses, vector<double> &x, vector<double> &y, vector<double> &yaw);
void sample_to_matlab_type(std::map<uint8_t, VehicleObservation> &sample, 
                          mgen::struct0_T (&vehiclePoses)[256], std::vector<uint8_t> &vehicle_ids);
int find_veh_index(vector<uint8_t> &vehicle_ids, int ego_vehicle_id);
