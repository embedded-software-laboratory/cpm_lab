#include "VehicleManualControl.hpp"
#include "example_trajectory.hpp"

VehicleManualControl::VehicleManualControl(shared_ptr<dds::domain::DomainParticipant> participant)
:participant(participant)
{
    topic_vehicleCommand = make_shared<dds::topic::Topic<VehicleCommand>>(*participant, "vehicleCommand");
    auto QoS = dds::pub::qos::DataWriterQos();
    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);
    writer_vehicleCommand = make_shared<dds::pub::DataWriter<VehicleCommand>>(
        dds::pub::Publisher(*participant), *topic_vehicleCommand, QoS);
}

void VehicleManualControl::start(uint8_t vehicleId, string joystick_device_file) 
{
    vehicle_id = vehicleId;
    joystick = make_shared<Joystick>(joystick_device_file);

    update_loop = make_shared<AbsoluteTimer>(0, 20000000, 0, 0, [&](){


        VehicleCommand sample;
        sample.vehicle_id(vehicle_id);

        if(joystick->getButton(4) || joystick->getButton(6)) { // constant speed mode
            sample.data()._d(VehicleCommandMode_def::SpeedCurvatureMode);

            double axis1 = joystick->getAxis(1) / (-double(1<<15));
            if(fabs(axis1) > 0.08) {
                ref_speed += axis1 * 0.02;
            }

            if(joystick->getButton(6)) {
                ref_speed = 1.0;
            }

            sample.data().speed_curvature().speed(ref_speed);
            sample.data().speed_curvature().curvature(joystick->getAxis(2) * 4.0 / (-double(1<<15)));

            printf("speed %12.4f  curvature %12.4f\n", 
                sample.data().speed_curvature().speed(), 
                sample.data().speed_curvature().curvature());

        }
        else if(joystick->getButton(5)) { // trajectory mode
            if(ref_trajectory_start_time == 0) {
                // start in 2 seconds
                ref_trajectory_start_time = clock_gettime_nanoseconds() + 2000000000ull; 
            }

            while(ref_trajectory_index < example_trajectory_size
                && ref_trajectory_start_time + example_trajectory_timestamp_offset[ref_trajectory_index] 
                    < clock_gettime_nanoseconds() + 1000000000ull) {

                ref_trajectory_index++;
            }
            std::cout << "ref_trajectory_index " << ref_trajectory_index << std::endl;


            sample.data()._d(VehicleCommandMode_def::TrajectoryMode);

            TrajectoryPoint trajectoryPoint;

            trajectoryPoint.t().nanoseconds(ref_trajectory_start_time + example_trajectory_timestamp_offset[ref_trajectory_index]);
            trajectoryPoint.px(example_trajectory_px[ref_trajectory_index]);
            trajectoryPoint.py(example_trajectory_py[ref_trajectory_index]);
            trajectoryPoint.vx(example_trajectory_vx[ref_trajectory_index]);
            trajectoryPoint.vy(example_trajectory_vy[ref_trajectory_index]);

            sample.data().trajectory_points(rti::core::vector<TrajectoryPoint>(1, trajectoryPoint));
        }
        else { // direct control
            sample.data()._d(VehicleCommandMode_def::DirectControlMode);
            sample.data().direct_control().motor_throttle(joystick->getAxis(1) / (-double(1<<15)));
            sample.data().direct_control().steering_servo(joystick->getAxis(2) / (-double(1<<15)));


            printf("motor_throttle %12.4f  steering_servo %12.4f\n", 
                sample.data().direct_control().motor_throttle(), 
                sample.data().direct_control().steering_servo());

        }


        // resets
        if(sample.data()._d() != VehicleCommandMode_def::SpeedCurvatureMode) {
            ref_speed = 0;
        }

        if(sample.data()._d() != VehicleCommandMode_def::TrajectoryMode) {
            ref_trajectory_start_time = 0;
            ref_trajectory_index = 0;
        }
        writer_vehicleCommand->write(sample);
    });
}


void VehicleManualControl::stop() 
{
    update_loop->stop();
    update_loop = nullptr;
    joystick = nullptr;
}
