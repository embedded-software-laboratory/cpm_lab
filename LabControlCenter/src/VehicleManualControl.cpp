#include "VehicleManualControl.hpp"
#include "example_trajectory.hpp"
#include "cpm/stamp_message.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"

#define AXIS_THROTTLE (1)
#define AXIS_STEERING (3)
#define BUTTON_SPEED_1MS (4)
#define BUTTON_SPEED_CONST (5)
#define BUTTON_TRAJECTORY (3)

VehicleManualControl::VehicleManualControl()
:participant(cpm::ParticipantSingleton::Instance())
,topic_vehicleCommandDirect(cpm::get_topic<VehicleCommandDirect>("vehicleCommandDirect"))
,topic_vehicleCommandSpeedCurvature(cpm::get_topic<VehicleCommandSpeedCurvature>("vehicleCommandSpeedCurvature"))
,topic_vehicleCommandTrajectory(cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory"))
{
    auto QoS = dds::pub::qos::DataWriterQos();
    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);
    auto publisher = dds::pub::Publisher(participant);
    publisher.default_datawriter_qos(QoS);


    writer_vehicleCommandDirect = make_shared<dds::pub::DataWriter<VehicleCommandDirect>>(publisher, topic_vehicleCommandDirect);
    writer_vehicleCommandSpeedCurvature = make_shared<dds::pub::DataWriter<VehicleCommandSpeedCurvature>>(publisher, topic_vehicleCommandSpeedCurvature);
    writer_vehicleCommandTrajectory = make_shared<dds::pub::DataWriter<VehicleCommandTrajectory>>(publisher, topic_vehicleCommandTrajectory);
}

void VehicleManualControl::start(uint8_t vehicleId, string joystick_device_file) 
{
    vehicle_id = vehicleId;
    joystick = make_shared<Joystick>(joystick_device_file);

    update_loop = make_shared<cpm::TimerFD>("LabControlCenter", 20000000ull, 0, false);

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
        else if(joystick->getButton(BUTTON_TRAJECTORY)) { // trajectory mode

            VehicleCommandTrajectory sample;
            sample.vehicle_id(vehicle_id);

            if(ref_trajectory_start_time == 0) {
                ref_trajectory_start_time = clock_gettime_nanoseconds() + 1000000000ull; 
            }

            while(ref_trajectory_index < example_trajectory_size
                && ref_trajectory_start_time + uint64_t(1e9*example_trajectory_timestamp_offset[ref_trajectory_index])
                    < clock_gettime_nanoseconds() + 500000000ull) {

                ref_trajectory_index++;
            }

            if(ref_trajectory_index >= example_trajectory_size)
            {
                ref_trajectory_index = 0;
                ref_trajectory_start_time += uint64_t(1e9*example_trajectory_period);
            }


            TrajectoryPoint trajectoryPoint;

            trajectoryPoint.t().nanoseconds(ref_trajectory_start_time + uint64_t(1e9*example_trajectory_timestamp_offset[ref_trajectory_index]));
            trajectoryPoint.px(example_trajectory_px[ref_trajectory_index]);
            trajectoryPoint.py(example_trajectory_py[ref_trajectory_index]);
            trajectoryPoint.vx(example_trajectory_vx[ref_trajectory_index]);
            trajectoryPoint.vy(example_trajectory_vy[ref_trajectory_index]);

            sample.trajectory_points(rti::core::vector<TrajectoryPoint>(1, trajectoryPoint));
            

            cpm::stamp_message(sample, t_now, 40000000ull);
            writer_vehicleCommandTrajectory->write(sample);
        }
        else { // direct control

            VehicleCommandDirect sample;
            sample.vehicle_id(vehicle_id);

            sample.motor_throttle(joystick->getAxis(AXIS_THROTTLE) / (-double(1<<15)));
            sample.steering_servo(joystick->getAxis(AXIS_STEERING) / (-double(1<<15)));

            cpm::stamp_message(sample, t_now, 40000000ull);
            writer_vehicleCommandDirect->write(sample);

            //printf("motor_throttle %12.4f  steering_servo %12.4f\n", 
            //    sample.data().direct_control().motor_throttle(), 
            //    sample.data().direct_control().steering_servo());


            // mode resets
            ref_speed = 0;
            ref_trajectory_start_time = 0;
            ref_trajectory_index = 0;

        }

        if(m_update_callback) m_update_callback();
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
        throttle = joystick->getAxis(AXIS_THROTTLE) / (-double(1<<15));
        steering = joystick->getAxis(AXIS_STEERING) / (-double(1<<15));
    }
}
