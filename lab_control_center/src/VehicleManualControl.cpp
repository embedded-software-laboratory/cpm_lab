#include "VehicleManualControl.hpp"
#include "cpm/stamp_message.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"

/**
 * \file VehicleManualControl.cpp
 * \ingroup lcc
 */

/**
 * \brief TODO
 * \ingroup lcc
 */
#define AXIS_THROTTLE (5)

/**
 * \brief TODO
 * \ingroup lcc
 */
#define AXIS_BRAKE (2)

/**
 * \brief TODO
 * \ingroup lcc
 */
#define AXIS_STEERING (0)

/**
 * \brief TODO
 * \ingroup lcc
 */
#define BUTTON_SPEED_1MS (4)

/**
 * \brief TODO
 * \ingroup lcc
 */
#define BUTTON_SPEED_CONST (5)

VehicleManualControl::VehicleManualControl()
:participant(cpm::ParticipantSingleton::Instance())
,topic_vehicleCommandDirect(cpm::get_topic<VehicleCommandDirect>("vehicleCommandDirect"))
,topic_vehicleCommandSpeedCurvature(cpm::get_topic<VehicleCommandSpeedCurvature>("vehicleCommandSpeedCurvature"))
{
    auto QoS = dds::pub::qos::DataWriterQos();
    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);
    auto publisher = dds::pub::Publisher(participant);
    publisher.default_datawriter_qos(QoS);


    writer_vehicleCommandDirect = make_shared<dds::pub::DataWriter<VehicleCommandDirect>>(publisher, topic_vehicleCommandDirect);
    writer_vehicleCommandSpeedCurvature = make_shared<dds::pub::DataWriter<VehicleCommandSpeedCurvature>>(publisher, topic_vehicleCommandSpeedCurvature);
}

void VehicleManualControl::start(uint8_t vehicleId, string joystick_device_file) 
{
    vehicle_id = vehicleId;
    joystick = make_shared<Joystick>(joystick_device_file);

    update_loop = std::make_shared<cpm::TimerFD>("lab_control_center", 20000000ull, 0, false);

    update_loop->start_async([&](uint64_t t_now){

        if(!joystick) return;


        if(joystick->getButton(BUTTON_SPEED_1MS) || joystick->getButton(BUTTON_SPEED_CONST)) { // constant speed mode

            VehicleCommandSpeedCurvature sample;
            sample.vehicle_id(vehicle_id);

            double axis1 = joystick->getAxis(AXIS_THROTTLE) / (-double(1<<15));
            if(fabs(axis1) > 0.08) {
                ref_speed += axis1 * 0.02;
            }

            if(joystick->getButton(BUTTON_SPEED_1MS)) {
                ref_speed = 1.0;
            }

            sample.speed(ref_speed);
            sample.curvature(joystick->getAxis(AXIS_STEERING) * 4.0 / (-double(1<<15)));

            //printf("speed %12.4f  curvature %12.4f\n", 
            //    sample.data().speed_curvature().speed(), 
            //    sample.data().speed_curvature().curvature());
            
            cpm::stamp_message(sample, t_now, 40000000ull);
            writer_vehicleCommandSpeedCurvature->write(sample);

        }
        else { // direct control

            VehicleCommandDirect sample;
            sample.vehicle_id(vehicle_id);

            double motor_throttle = 0;
            double steering_servo = 0;
            get_state(motor_throttle, steering_servo);
            sample.motor_throttle(motor_throttle);
            sample.steering_servo(steering_servo);

            cpm::stamp_message(sample, t_now, 40000000ull);
            writer_vehicleCommandDirect->write(sample);

            // mode reset
            ref_speed = 0;

        }

        if(m_update_callback) m_update_callback();
    },
    [](){
        //Empty lambda callback for stop signals -> Do nothing when a stop signal is received
    });
}


void VehicleManualControl::stop() 
{
    if(update_loop) {        
        update_loop->stop();
        update_loop = nullptr;
    }
    joystick = nullptr;
}


void VehicleManualControl::get_state(double& throttle, double& steering) 
{
    if(joystick) {
        throttle = (joystick->getAxis(AXIS_THROTTLE) - joystick->getAxis(AXIS_BRAKE)) / (double(1<<16));
        steering = joystick->getAxis(AXIS_STEERING) / (-double(1<<15));
    }
}
