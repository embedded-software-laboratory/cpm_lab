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
:topic_vehiclePoseSimulated(cpm::ParticipantSingleton::Instance(), "vehiclePoseSimulated")
,writer_vehiclePoseSimulated(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), topic_vehiclePoseSimulated)
,simulationIPS(_simulationIPS)
{
    // fill a list with empty actuation inputs to simulate delay
    ActuatorInput input;
    for (size_t i = 0; i < input_delay; ++i)
    {
        actuator_inputs.push_back(input);
    }
    
}

VehicleState SimulationVehicle::update(
        const double& motor_throttle,
        const double& steering_servo,
        const uint8_t& motor_mode,
        const uint64_t& t_now, 
        const double& dt, 
        const uint8_t& vehicle_id
)
{
    actuator_inputs.push_back(
        ActuatorInput(motor_throttle, steering_servo, motor_mode));

    const double T_curvature = 0.09; // steering PT1 time constant in seconds
    const double T_speed = 0.7;      // speed PT1 time constant in seconds
    // TODO: what are these equations
    double speed_ref = 6.5 * pow(actuator_inputs.front().motor_throttle, 1.6);
    double curvature_ref = (actuator_inputs.front().steering_servo + 0.035)/(0.24);

    // kinematic bounds on turning radius
    curvature_ref = fmin( 2.8, curvature_ref);
    curvature_ref = fmax(-2.8, curvature_ref);

    // TODO: First motor_mode check, then clipping in positive direction?
    // Max speed is 4 m/s (TODO: Check)
    speed_ref = fmin( 4.0, speed_ref);
    speed_ref = fmax(-4.0, speed_ref);
    // bounds on speed depending on turning radius to avoid slip
    const double speed_max_acc = sqrt(8/(fabs(curvature_ref)+0.01));
    speed_ref = fmin( speed_max_acc, speed_ref);
    speed_ref = fmax(-speed_max_acc, speed_ref);
    if(actuator_inputs.front().motor_mode == SPI_MOTOR_MODE_BRAKE) 
    {
        speed_ref = 0;
    }
    else if(actuator_inputs.front().motor_mode == SPI_MOTOR_MODE_REVERSE) 
    {
        speed_ref = -speed_ref;
    }

    // remove used inputs from list
    actuator_inputs.pop_front();

    // solve ODE timestep
    const int N_substeps = 5;
    for (int i = 0; i < N_substeps; ++i)
    {
        speed +=         (dt/N_substeps) * ((speed_ref-speed)/T_speed);
        curvature +=     (dt/N_substeps) * ((curvature_ref-curvature)/T_curvature);
        yaw +=           (dt/N_substeps) * (curvature*speed);
        yaw_measured +=  (dt/N_substeps) * (curvature*speed);
        x +=             (dt/N_substeps) * (speed*cos(yaw));
        y +=             (dt/N_substeps) * (speed*sin(yaw));
        distance +=      (dt/N_substeps) * (speed);
    }

    yaw_measured += 1e-4 * frand(); // simulate random biased gyro drift
    if(yaw_measured > 2*M_PI) yaw_measured -= 2*M_PI;
    if(yaw_measured < 0)      yaw_measured += 2*M_PI;

    // Publish simulated state
    {
        VehicleObservation simulatedState;
        simulatedState.vehicle_id(vehicle_id);
        simulatedState.pose().x(x);
        simulatedState.pose().y(y);
        simulatedState.pose().yaw( remainder(yaw, 2*M_PI) ); // yaw in range [-PI, PI]
        cpm::stamp_message(simulatedState, t_now, 0);
        writer_vehiclePoseSimulated.write(simulatedState);

        simulationIPS.update(simulatedState);
    }

    /*std::cout 
    << "curvature_ref"         << "  " << curvature_ref         << std::endl
    << "speed_ref"             << "  " << speed_ref             << std::endl
    << "dt"                    << "  " << dt                    << std::endl
    << "speed"                 << "  " << speed                 << std::endl
    << "curvature"             << "  " << curvature             << std::endl
    << "yaw"                   << "  " << yaw                   << std::endl
    << "spi_miso_data.imu_yaw" << "  " << spi_miso_data.imu_yaw << std::endl
    << "x"                     << "  " << x                     << std::endl
    << "distance"              << "  " << distance              << std::endl
    << "===============================" << std::endl;*/

    VehicleState vehicleState;
    vehicleState.odometer_distance           (distance);
    vehicleState.pose().x                    (0); // Not measured, TBD by the localization
    vehicleState.pose().y                    (0);
    vehicleState.pose().yaw                  (0);
    vehicleState.imu_acceleration_forward    ((speed_ref-speed)/T_speed);
    vehicleState.imu_acceleration_left       (curvature*speed*speed);
    vehicleState.imu_acceleration_up         (0);
    vehicleState.imu_yaw                     (yaw_measured);
    vehicleState.imu_yaw_rate                (curvature*speed);
    vehicleState.speed                       (speed);
    vehicleState.battery_voltage             (7.2 - 0.2 * fabs(speed_ref-speed));
    vehicleState.motor_current               (fabs((speed_ref-speed) * 0.2));
    return vehicleState;
}




void SimulationVehicle::get_state(double& _x, double& _y, double& _yaw, double& _speed) 
{
    _x = x;
    _y = y;
    _yaw = yaw;
    _speed = speed;
}