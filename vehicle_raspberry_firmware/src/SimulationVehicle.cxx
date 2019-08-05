#include "SimulationVehicle.hpp"
#include <string.h>
#include <math.h>
#include <iostream>
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/stamp_message.hpp"

extern "C" {
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.h"
}



SimulationVehicle::SimulationVehicle(SimulationIPS& _simulationIPS)
:topic_vehiclePoseSimulated(cpm::get_topic<VehicleObservation>("vehiclePoseSimulated"))
,writer_vehiclePoseSimulated(
    dds::pub::Publisher(cpm::ParticipantSingleton::Instance()),
    topic_vehiclePoseSimulated)
,simulationIPS(_simulationIPS)
{      
    dds::sub::Subscriber subscriber = dds::sub::Subscriber(cpm::ParticipantSingleton::Instance());
    dds::sub::qos::DataReaderQos qos = 
        (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll());
    reader_vehicle_observation = 
        dds::sub::DataReader<VehicleObservation>(subscriber, topic_vehiclePoseSimulated, qos);
}


void SimulationVehicle::check_for_collision(const uint8_t vehicle_id)
{
    for(auto sample : reader_vehicle_observation.take())
    {
        if(sample.info().valid())
        {
            auto data = sample.data();
            // if id == own id continue
        }
    }
}

bool SimulationVehicle::is_collision(
    const uint64_t timestamp,
    const Pose2D pose2D)
{

}

bool are_rectangle_intersecting(const Pose2D pose2D_1, const Pose2D pose2D_2)
{
    // vehicle geometry from reference point
    double l_front = 0.12;
// l_back:  0.101
// w_left:  0.55
// w_right: 0.55

}

VehicleState SimulationVehicle::update(
        const double& motor_throttle,
        const double& steering_servo,
        const uint64_t& t_now, 
        const double& dt, 
        const uint8_t& vehicle_id
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
    // Check for collision
    check_for_collision(vehicle_id);
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
    vehicleState.battery_voltage             (7.8 - 0.2 * fabs(d_speed));
    vehicleState.motor_current               (fabs((d_speed) * 0.2));
    return vehicleState;
}


void SimulationVehicle::get_state(double& _px, double& _py, double& _yaw, double& _speed) 
{
    _px = px;
    _py = py;
    _yaw = yaw;
    _speed = speed;
}