// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#include "SimulationVehicle.hpp"
#include <string.h>
#include <math.h>
#include <iostream>

extern "C" {
#include "../../low_level_controller/vehicle_atmega2560_firmware/crc.h"
}

SimulationVehicle::SimulationVehicle(SimulationIPS& _simulationIPS, uint8_t vehicle_id, vector<double> starting_position)
:topic_vehiclePoseSimulated(cpm::get_topic<VehicleObservation>("vehiclePoseSimulated"))
,writer_vehiclePoseSimulated(
    dds::pub::Publisher(cpm::ParticipantSingleton::Instance()),
    topic_vehiclePoseSimulated)
,reader_vehiclePoseSimulated(topic_vehiclePoseSimulated, MAX_NUM_VEHICLES)
,simulationIPS(_simulationIPS)
{
    // select a starting position on the "map2" layout
    const std::vector<double> nodes_x = std::vector<double>{2.2500e+00,3.1500e+00,2.2500e+00,3.1575e+00,3.8074e+00,3.8795e+00,4.2103e+00,4.3584e+00,4.2450e+00,4.3950e+00,3.8956e+00,4.0623e+00,3.8242e+00,3.0500e+00,3.0500e+00,2.3250e+00,2.4884e+00,2.4750e+00,2.6065e+00,3.1425e+00,2.2500e+00,2.4750e+00,2.1750e+00,3.0500e+00,2.0250e+00,3.0500e+00,2.3250e+00,2.2500e+00,1.3500e+00,1.3425e+00,6.9264e-01,6.2050e-01,2.8966e-01,1.4158e-01,2.5500e-01,1.0500e-01,6.0440e-01,6.7576e-01,4.3775e-01,1.4500e+00,1.4500e+00,2.0116e+00,1.8935e+00,1.3575e+00,2.0250e+00,1.4500e+00,1.4500e+00,2.1750e+00,3.1500e+00,2.2500e+00,3.1575e+00,2.2500e+00,3.8074e+00,3.8795e+00,4.2103e+00,4.3584e+00,3.8956e+00,3.8242e+00,4.0623e+00,2.4884e+00,2.3250e+00,2.6065e+00,2.4750e+00,3.1425e+00,2.2500e+00,2.1750e+00,2.0250e+00,2.2500e+00,1.3500e+00,1.3425e+00,6.9264e-01,6.2050e-01,2.8966e-01,1.4158e-01,6.0440e-01,4.3775e-01,6.7576e-01,2.0116e+00,1.8935e+00,1.3575e+00,};
    const std::vector<double> nodes_y = std::vector<double>{3.7450e+00,3.7390e+00,3.8950e+00,3.8888e+00,3.5902e+00,3.7217e+00,2.9310e+00,2.9549e+00,2.0000e+00,2.0000e+00,2.1939e+00,2.9071e+00,2.3258e+00,2.0750e+00,2.2250e+00,2.8000e+00,3.5006e+00,2.8000e+00,3.4081e+00,3.5892e+00,2.2250e+00,2.0000e+00,2.8000e+00,1.9250e+00,2.8000e+00,1.7750e+00,2.0000e+00,2.0750e+00,3.7390e+00,3.8888e+00,3.5902e+00,3.7217e+00,2.9310e+00,2.9549e+00,2.0000e+00,2.0000e+00,2.1939e+00,2.3258e+00,2.9071e+00,2.0750e+00,2.2250e+00,3.5006e+00,3.4081e+00,3.5892e+00,2.0000e+00,1.9250e+00,1.7750e+00,2.0000e+00,2.6100e-01,2.5500e-01,1.1119e-01,1.0500e-01,4.0979e-01,2.7828e-01,1.0690e+00,1.0451e+00,1.8061e+00,1.6742e+00,1.0929e+00,4.9935e-01,1.2000e+00,5.9189e-01,1.2000e+00,4.1081e-01,1.7750e+00,1.2000e+00,1.2000e+00,1.9250e+00,2.6100e-01,1.1119e-01,4.0979e-01,2.7828e-01,1.0690e+00,1.0451e+00,1.8061e+00,1.0929e+00,1.6742e+00,4.9935e-01,5.9189e-01,4.1081e-01,};
    const std::vector<double> nodes_cos = std::vector<double>{1.0000e+00,9.9875e-01,1.0000e+00,9.9874e-01,8.7677e-01,8.7680e-01,1.5918e-01,1.5927e-01,0.0000e+00,0.0000e+00,-8.7962e-01,1.5925e-01,-8.7962e-01,-1.0000e+00,-1.0000e+00,4.0366e-05,6.1691e-01,-4.0327e-05,6.1691e-01,9.9874e-01,-1.0000e+00,0.0000e+00,4.0366e-05,1.0000e+00,-4.0327e-05,1.0000e+00,0.0000e+00,-1.0000e+00,9.9875e-01,9.9874e-01,8.7677e-01,8.7680e-01,1.5918e-01,1.5927e-01,0.0000e+00,0.0000e+00,-8.7962e-01,-8.7962e-01,1.5925e-01,-1.0000e+00,-1.0000e+00,6.1691e-01,6.1691e-01,9.9874e-01,0.0000e+00,1.0000e+00,1.0000e+00,0.0000e+00,-9.9875e-01,-1.0000e+00,-9.9874e-01,-1.0000e+00,-8.7677e-01,-8.7680e-01,-1.5918e-01,-1.5927e-01,8.7962e-01,8.7962e-01,-1.5925e-01,-6.1691e-01,-4.0366e-05,-6.1691e-01,4.0327e-05,-9.9874e-01,1.0000e+00,-4.0366e-05,4.0327e-05,1.0000e+00,-9.9875e-01,-9.9874e-01,-8.7677e-01,-8.7680e-01,-1.5918e-01,-1.5927e-01,8.7962e-01,-1.5925e-01,8.7962e-01,-6.1691e-01,-6.1691e-01,-9.9874e-01,};
    const std::vector<double> nodes_sin = std::vector<double>{0.0000e+00,-5.0058e-02,0.0000e+00,-5.0154e-02,-4.8090e-01,-4.8086e-01,-9.8725e-01,-9.8724e-01,-1.0000e+00,-1.0000e+00,-4.7568e-01,-9.8724e-01,-4.7568e-01,-3.4318e-05,3.4305e-05,1.0000e+00,7.8703e-01,1.0000e+00,7.8703e-01,-5.0169e-02,0.0000e+00,1.0000e+00,-1.0000e+00,-3.4318e-05,-1.0000e+00,3.4305e-05,1.0000e+00,0.0000e+00,5.0058e-02,5.0154e-02,4.8090e-01,4.8086e-01,9.8725e-01,9.8724e-01,1.0000e+00,1.0000e+00,4.7568e-01,4.7568e-01,9.8724e-01,3.4318e-05,-3.4305e-05,-7.8703e-01,-7.8703e-01,5.0169e-02,-1.0000e+00,3.4318e-05,-3.4305e-05,-1.0000e+00,-5.0058e-02,0.0000e+00,-5.0154e-02,0.0000e+00,-4.8090e-01,-4.8086e-01,-9.8725e-01,-9.8724e-01,-4.7568e-01,-4.7568e-01,-9.8724e-01,7.8703e-01,1.0000e+00,7.8703e-01,1.0000e+00,-5.0169e-02,0.0000e+00,-1.0000e+00,-1.0000e+00,0.0000e+00,5.0058e-02,5.0154e-02,4.8090e-01,4.8086e-01,9.8725e-01,9.8724e-01,4.7568e-01,9.8724e-01,4.7568e-01,-7.8703e-01,-7.8703e-01,5.0169e-02,};
    const std::vector<double> index_map {2,3,5,7,9,55,53,50,51,69,71,73,35,33,31,29,14,25,62,66,46,40,24,17};  
    if(starting_position.size() != 3){
        //TODO: Change method, this is not valid for vehicle IDs > 23
        px = nodes_x.at(index_map.at(vehicle_id));
        py = nodes_y.at(index_map.at(vehicle_id));
        yaw = atan2(nodes_sin.at(index_map.at(vehicle_id)), nodes_cos.at(index_map.at(vehicle_id)));
        yaw_measured = yaw;
    } else {
        px = starting_position[0];
        py = starting_position[1];
        yaw = starting_position[2];
        yaw_measured = starting_position[2]; 
    }
}

VehicleState SimulationVehicle::update(
    const double motor_throttle,
    const double steering_servo,
    const uint64_t t_now, 
    const double dt, 
    const uint8_t vehicle_id
)
{
    // account for input delay
    double cur_motor_throttle;
    double cur_steering_servo;
#if INPUT_DELAY==0
    cur_motor_throttle = motor_throttle;
    cur_steering_servo = steering_servo;
#else
    cur_motor_throttle = motor_throttle_history[0];
    cur_steering_servo = steering_servo_history[0];
    for (size_t i = 1; i < INPUT_DELAY; ++i)
    {
        motor_throttle_history[i-1] = motor_throttle_history[i];
        steering_servo_history[i-1] = steering_servo_history[i];
    }
    motor_throttle_history[INPUT_DELAY-1] = motor_throttle;
    steering_servo_history[INPUT_DELAY-1] = steering_servo;
#endif

    // solve ODE timestep
    double d_px, d_py, d_yaw, d_speed;
    VehicleModel::step(
        dynamics_parameters,
        dt,
        cur_motor_throttle,
        cur_steering_servo,
        7.8,
        px, py, yaw, speed,
        d_px, d_py, d_yaw, d_speed
    );

    distance += dt * speed;
    yaw_measured += dt * d_yaw + 1e-4 * frand(); // simulate random biased gyro drift
    yaw_measured = remainder(yaw_measured, 2*M_PI); // yaw in range [-PI, PI]


    // Publish simulated state
    {
        VehicleObservation simulatedState;
        simulatedState.vehicle_id(vehicle_id);
        simulatedState.pose().x(px);
        simulatedState.pose().y(py);
        simulatedState.pose().yaw( remainder(yaw, 2*M_PI) ); // yaw in range [-PI, PI]
        cpm::stamp_message(simulatedState, t_now, 0);
        writer_vehiclePoseSimulated.write(simulatedState);

        simulationIPS.update(simulatedState);
    }

    
    // save current pose
    ego_pose_history[t_now] = Pose2D(px, py, yaw);
    // Check for collision
    std::map<uint8_t, uint64_t> collisions = get_collisions(t_now, vehicle_id);
    for (auto const& colli : collisions)
    {
        cpm::Logging::Instance().write(
            1,
            "Warning: Simulation: Collision with vehicle %u at time %llu.", 
            colli.first, colli.second);
    }
    // Erase trajectory points which are older than 0.5 seconds
    const uint64_t past_threshold_time = t_now - 500000000ull;
    auto last_valid_it = ego_pose_history.upper_bound(past_threshold_time);
    ego_pose_history.erase(ego_pose_history.begin(), last_valid_it);

    /*std::cout 
    << "dt"                    << "  " << dt                    << std::endl
    << "speed"                 << "  " << speed                 << std::endl
    << "curvature"             << "  " << curvature             << std::endl
    << "yaw"                   << "  " << yaw                   << std::endl
    << "x"                     << "  " << x                     << std::endl
    << "distance"              << "  " << distance              << std::endl
    << "===============================" << std::endl;*/

    VehicleState vehicleState;
    vehicleState.odometer_distance           (distance);
    vehicleState.pose().x                    (0); // Not measured, TBD by the localization
    vehicleState.pose().y                    (0);
    vehicleState.pose().yaw                  (0);
    vehicleState.imu_acceleration_forward    (d_speed);
    vehicleState.imu_acceleration_left       (d_yaw*speed);
    vehicleState.imu_acceleration_up         (0);
    vehicleState.imu_yaw                     (yaw_measured);
    vehicleState.imu_yaw_rate                (d_yaw);
    vehicleState.speed                       (speed);
    vehicleState.battery_voltage             (7.8 - 0.03 * fabs(d_speed));
    vehicleState.motor_current               (fabs((d_speed) * 0.2));
    vehicleState.is_real                     (false); //Is not real, is simulated
    return vehicleState;
}


void SimulationVehicle::get_state(double& _px, double& _py, double& _yaw, double& _speed) 
{
    _px = px;
    _py = py;
    _yaw = yaw;
    _speed = speed;
}


std::map<uint8_t, uint64_t>  SimulationVehicle::get_collisions(
    const uint64_t t_now,
    const uint8_t vehicle_id
)
{
    std::map<uint8_t, uint64_t> collisions;
    
    std::map<uint8_t, VehicleObservation> sample_out; 
    std::map<uint8_t, uint64_t> sample_age_out;
    reader_vehiclePoseSimulated.get_samples(t_now, sample_out, sample_age_out);
    for(const auto& entry : sample_out)
    {
        const auto vehicle_id_in = entry.first;
        const auto sample = entry.second;
        // don't consider samples older than 1s
        if (sample.header().create_stamp().nanoseconds() + 1000000000ull < t_now) continue;
        if (vehicle_id_in == vehicle_id) continue; // don't consider ego vehicle
        // find ego pose corresponding to sample time
        auto ego_pose_it = ego_pose_history.find(sample.header().create_stamp().nanoseconds());
        if (ego_pose_it == ego_pose_history.end()) continue;
        // check for collision
        PathNode nodeA = PathNode(
            ego_pose_it->second.x(),
            ego_pose_it->second.y(),
            cos(ego_pose_it->second.yaw()),
            sin(ego_pose_it->second.yaw())
        );
        PathNode nodeB = PathNode(
            sample.pose().x(),
            sample.pose().y(),
            cos(sample.pose().yaw()),
            sin(sample.pose().yaw())
        );
        const double distance = min_distance_vehicle_to_vehicle(nodeA, nodeB);
        if (distance < 0.001)
        {
            // add collision to result
            collisions[vehicle_id_in] = sample.header().create_stamp().nanoseconds();
        }
    }
    return collisions;
}