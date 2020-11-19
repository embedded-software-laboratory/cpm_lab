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

#include "Controller.hpp"
#include <iostream>
#include "cpm/Parameter.hpp"
#include "cpm/Logging.hpp"
#include "cpm/TimeMeasurement.hpp"

using namespace std::placeholders;

Controller::Controller(uint8_t _vehicle_id, std::function<uint64_t()> _get_time)
:
mpcController(_vehicle_id, std::bind(&Controller::get_stop_signals, this, _1, _2))
,m_get_time(_get_time)
,topic_vehicleCommandDirect(cpm::VehicleIDFilteredTopic<VehicleCommandDirect>(cpm::get_topic<VehicleCommandDirect>("vehicleCommandDirect"), _vehicle_id))
,topic_vehicleCommandSpeedCurvature(cpm::VehicleIDFilteredTopic<VehicleCommandSpeedCurvature>(cpm::get_topic<VehicleCommandSpeedCurvature>("vehicleCommandSpeedCurvature"), _vehicle_id))
,topic_vehicleCommandTrajectory(cpm::VehicleIDFilteredTopic<VehicleCommandTrajectory>(cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory"), _vehicle_id))
,vehicle_id(_vehicle_id)
{
    reader_CommandDirect = std::unique_ptr<cpm::Reader<VehicleCommandDirect>>(new cpm::Reader<VehicleCommandDirect>(topic_vehicleCommandDirect));
    reader_CommandSpeedCurvature = std::unique_ptr<cpm::Reader<VehicleCommandSpeedCurvature>>(new cpm::Reader<VehicleCommandSpeedCurvature>(topic_vehicleCommandSpeedCurvature));
    reader_CommandTrajectory = std::unique_ptr<cpm::Reader<VehicleCommandTrajectory>>(new cpm::Reader<VehicleCommandTrajectory>(topic_vehicleCommandTrajectory));
}


void Controller::update_vehicle_state(VehicleState vehicleState) 
{
    m_vehicleState = vehicleState;
}


void Controller::receive_commands(uint64_t t_now)
{
    std::lock_guard<std::mutex> lock(command_receive_mutex);

    VehicleCommandDirect sample_CommandDirect;
    uint64_t sample_CommandDirect_age;

    VehicleCommandSpeedCurvature sample_CommandSpeedCurvature;
    uint64_t sample_CommandSpeedCurvature_age;

    VehicleCommandTrajectory sample_CommandTrajectory;
    uint64_t sample_CommandTrajectory_age;

    reader_CommandDirect->get_sample(t_now, sample_CommandDirect, sample_CommandDirect_age);
    reader_CommandSpeedCurvature->get_sample(t_now, sample_CommandSpeedCurvature, sample_CommandSpeedCurvature_age);
    reader_CommandTrajectory->get_sample(t_now, sample_CommandTrajectory, sample_CommandTrajectory_age);

    if(sample_CommandDirect_age < command_timeout)
    {
        m_vehicleCommandDirect = sample_CommandDirect;
        state = ControllerState::Direct;

        //Evaluation: Log received timestamp
        cpm::Logging::Instance().write(
            3,
            "Controller: Read direct message. "
            "Valid after %llu.",
            sample_CommandDirect.header().valid_after_stamp().nanoseconds()
        );
    }
    else if(sample_CommandSpeedCurvature_age < command_timeout)
    {
        m_vehicleCommandSpeedCurvature = sample_CommandSpeedCurvature;  
        state = ControllerState::SpeedCurvature;

        //Evaluation: Log received timestamp
        cpm::Logging::Instance().write(
            3,
            "Controller: Read speed curvature message. "
            "Valid after %llu",
            sample_CommandSpeedCurvature.header().valid_after_stamp().nanoseconds()
        );
    }
    else if (sample_CommandTrajectory_age < command_timeout)
    {
        m_vehicleCommandTrajectory = sample_CommandTrajectory;  
        state = ControllerState::Trajectory;

        //Evaluation: Log received timestamp
        cpm::Logging::Instance().write(
            3,
            "Controller: Read trajectory message. "
            "Valid after %llu",
            sample_CommandTrajectory.header().valid_after_stamp().nanoseconds()
        );
    }
    // no new commands received
    else if (state != ControllerState::Stop)
    {
        state = ControllerState::Stop;
        //Use %s, else we get a warning that this is no string literal (we do not want unnecessary warnings to show up)
        cpm::Logging::Instance().write(
            1,
            "Warning: Controller: "
            "No new commands received. %s", "Stopping."
        );
    }
}

double Controller::speed_controller(const double speed_measured, const double speed_target) 
{

    // steady-state curve, from curve fitting
    double motor_throttle = ((speed_target>=0)?(1.0):(-1.0)) * pow(fabs(0.152744 * speed_target),(0.627910));

    const double speed_error = speed_target - speed_measured;

    // PI controller for the speed
    const double integral_gain = 0.01;
    const double proportional_gain = 0.3;
    speed_throttle_error_integral += integral_gain * speed_error;

    speed_throttle_error_integral = fmin(0.5, fmax(-0.5, speed_throttle_error_integral)); // integral clamping

    motor_throttle += speed_throttle_error_integral;
    motor_throttle += proportional_gain * speed_error;
    return motor_throttle;
}


double steering_curvature_calibration(double curvature) 
{
    // steady-state curve, from curve fitting
    double steering_servo = (0.241857) * curvature;
    return steering_servo;
}


void Controller::update_remote_parameters()
{
    //trajectory_controller_lateral_P_gain = cpm::parameter_double("trajectory_controller/lateral_P_gain");
    //trajectory_controller_lateral_D_gain = cpm::parameter_double("trajectory_controller/lateral_D_gain");
}


std::shared_ptr<TrajectoryInterpolation> Controller::interpolate_trajectory_command(uint64_t t_now)
{
    if (m_vehicleCommandTrajectory.trajectory_points().size() < 2) return nullptr;
    //m_vehicleCommandTrajectory is updated in receive_commands, which gets called in get_control_signals
    //The reason for this confusing structure is that it was most compatible to the already existing solution for the other data types
    if(m_vehicleCommandTrajectory.header().create_stamp().nanoseconds() > 0) 
    {
        //Get current segment (trajectory points) in current trajectory for interpolation
        auto start_point = TrajectoryPoint();
        auto end_point = TrajectoryPoint();
        start_point.t().nanoseconds(0);
        end_point.t().nanoseconds(0);

        //When looking up the current segment, start at 1, because start and end must follow each other (we look up end, and from that determine start)
        for (size_t i = 1; i < m_vehicleCommandTrajectory.trajectory_points().size(); ++i)
        {
            if (m_vehicleCommandTrajectory.trajectory_points().at(i).t().nanoseconds() >= t_now)
            {
                end_point = m_vehicleCommandTrajectory.trajectory_points().at(i);
                start_point = m_vehicleCommandTrajectory.trajectory_points().at(i - 1);
                break;
            }
        }

        //Log an error if we could not find a valid trajectory segment w.r.t. end
        if (end_point.t().nanoseconds() == 0)
        {
            cpm::Logging::Instance().write(
                2,
                "Warning: Controller: %s",
                "Trajectory interpolation error: Missing trajectory point in the FUTURE."
            );
            return nullptr;
        }

        //Log an error if we could not find a valid trajectory segment w.r.t. start
        if (start_point.t().nanoseconds() >= t_now)
        {
            cpm::Logging::Instance().write(
                2,
                "Warning: Controller: %s",
                "Trajectory interpolation error: Missing trajectory point in the PAST."
            );
            return nullptr;
        }
    
        assert(t_now >= start_point.t().nanoseconds());
        assert(t_now <= end_point.t().nanoseconds());

        // We have a valid trajectory segment.
        // Interpolate for the current time.
        return std::make_shared<TrajectoryInterpolation>(t_now, start_point, end_point);
    }
    else 
    {
        cpm::Logging::Instance().write(
            2,
            "Warning: Controller: %s",
            "Trajectory interpolation error: No valid trajectory data."
        );
    }
    return nullptr;
}


void Controller::trajectory_controller_linear(uint64_t t_now, double &motor_throttle_out, double &steering_servo_out)
{
    std::shared_ptr<TrajectoryInterpolation> trajectory_interpolation = 
        interpolate_trajectory_command(t_now);

    if(trajectory_interpolation) 
    {
        const double x_ref = trajectory_interpolation->position_x;
        const double y_ref = trajectory_interpolation->position_y;
        const double yaw_ref = trajectory_interpolation->yaw;

        const double x = m_vehicleState.pose().x();
        const double y = m_vehicleState.pose().y();
        const double yaw = m_vehicleState.pose().yaw();

        double longitudinal_error =  cos(yaw_ref) * (x-x_ref)  + sin(yaw_ref) * (y-y_ref);
        double lateral_error      = -sin(yaw_ref) * (x-x_ref)  + cos(yaw_ref) * (y-y_ref);
        const double yaw_error = sin(yaw - yaw_ref);



        if(fabs(lateral_error) < 0.8 && fabs(longitudinal_error) < 0.8 && fabs(yaw_error) < 0.7)
        {
            // Linear lateral controller
            const double ref_curvature = fmin(0.5,fmax(-0.5,trajectory_interpolation->curvature));
            //const double ref_curvature = trajectory_interpolation->curvature;
            const double curvature = ref_curvature 
                - trajectory_controller_lateral_P_gain * lateral_error 
                - trajectory_controller_lateral_D_gain * yaw_error;

            // Linear longitudinal controller
            const double speed_target = trajectory_interpolation->speed - 0.5 * longitudinal_error;

            const double speed_measured = m_vehicleState.speed();
            steering_servo_out = steering_curvature_calibration(curvature);
            motor_throttle_out = speed_controller(speed_measured, speed_target);


            /*std::cout << 
            "lateral_error " << lateral_error << "  " << 
            "longitudinal_error " << longitudinal_error << "  " << 
            "yaw_error " << yaw_error << "  " << 
            "ref_curvature " << trajectory_interpolation->curvature << "  " << 
            "curvature_cmd " << curvature << "  " << 
            std::endl;*/
        }
    }
}

static inline double square(double x) {return x*x;}

void Controller::trajectory_tracking_statistics_update(uint64_t t_now)
{
    std::shared_ptr<TrajectoryInterpolation> trajectory_interpolation = 
        interpolate_trajectory_command(t_now);

    if(trajectory_interpolation) 
    {
        const double x_ref = trajectory_interpolation->position_x;
        const double y_ref = trajectory_interpolation->position_y;
        const double yaw_ref = trajectory_interpolation->yaw;

        const double x = m_vehicleState.pose().x();
        const double y = m_vehicleState.pose().y();
        //const double yaw = m_vehicleState.pose().yaw();

        double longitudinal_error =  cos(yaw_ref) * (x-x_ref)  + sin(yaw_ref) * (y-y_ref);
        double lateral_error      = -sin(yaw_ref) * (x-x_ref)  + cos(yaw_ref) * (y-y_ref);

        trajectory_tracking_statistics_longitudinal_errors[trajectory_tracking_statistics_index] = longitudinal_error;
        trajectory_tracking_statistics_lateral_errors     [trajectory_tracking_statistics_index] = lateral_error;

        trajectory_tracking_statistics_index = (trajectory_tracking_statistics_index+1) 
                                                % TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE;

        if(trajectory_tracking_statistics_index == 0)
        {
            // output summary

            double longitudinal_error_sum = 0;
            double lateral_error_sum = 0;
            double longitudinal_error_max = 0;
            double lateral_error_max = 0;

            for (int i = 0; i < TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE; ++i)
            {
                longitudinal_error_sum += trajectory_tracking_statistics_longitudinal_errors[i];
                lateral_error_sum += trajectory_tracking_statistics_lateral_errors[i];

                longitudinal_error_max = fmax(longitudinal_error_max, fabs(trajectory_tracking_statistics_longitudinal_errors[i]));
                lateral_error_max = fmax(lateral_error_max, fabs(trajectory_tracking_statistics_lateral_errors[i]));
            }

            const double longitudinal_error_mean = longitudinal_error_sum / TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE;
            const double lateral_error_mean = lateral_error_sum / TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE;


            double longitudinal_error_variance_sum = 0;
            double lateral_error_variance_sum = 0;

            for (int i = 0; i < TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE; ++i)
            {
                longitudinal_error_variance_sum += square(trajectory_tracking_statistics_longitudinal_errors[i] - longitudinal_error_mean);
                lateral_error_variance_sum += square(trajectory_tracking_statistics_lateral_errors[i] - lateral_error_mean);
            }


            const double longitudinal_error_variance = longitudinal_error_variance_sum / (TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE - 1);
            const double lateral_error_variance = lateral_error_variance_sum / (TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE - 1);

            const double longitudinal_error_std = sqrt(longitudinal_error_variance);
            const double lateral_error_std = sqrt(lateral_error_variance);

            cpm::Logging::Instance().write(
                3,
                "Vehicle Controller Tracking Errors:"
                "long,mean: %f  "
                "long,std: %f  "
                "long,max: %f  "
                "lat,mean: %f  "
                "lat,std: %f  "
                "lat,max: %f  ",
                longitudinal_error_mean, 
                longitudinal_error_std,
                longitudinal_error_max,
                lateral_error_mean,
                lateral_error_std,
                lateral_error_max
            );
        }
    }
}

void Controller::get_control_signals(uint64_t t_now, double &out_motor_throttle, double &out_steering_servo) 
{
    // cpm::TimeMeasurement::Instance().start("mpc_start");
    receive_commands(t_now);

    update_remote_parameters();


    double motor_throttle = 0;
    double steering_servo = 0;

    if(m_vehicleState.IPS_update_age_nanoseconds() > 3000000000ull 
        && state == ControllerState::Trajectory)
    {
        //Use %s, else we get a warning that this is no string literal (we do not want unnecessary warnings to show up)
        cpm::Logging::Instance().write(
            1,
            "Error: Controller: "
            "Lost IPS position reference. %s", "Stopping.");

        state = ControllerState::Stop;
    }
    // cpm::TimeMeasurement::Instance().stop("mpc_start");
    // cpm::TimeMeasurement::Instance().start("mpc_switch");

    switch(state) {

        case ControllerState::Stop:
        {
            // Use function that calculates motor values for stopping immediately - which is also already used in main
            get_stop_signals(motor_throttle, steering_servo);
        }
        break;

        case ControllerState::SpeedCurvature:
        {
            const double speed_target = m_vehicleCommandSpeedCurvature.speed();
            const double curvature    = m_vehicleCommandSpeedCurvature.curvature();
            const double speed_measured = m_vehicleState.speed();

            steering_servo = steering_curvature_calibration(curvature);
            motor_throttle = speed_controller(speed_measured, speed_target);
        }
        break;

        case ControllerState::Trajectory:
        {
            std::lock_guard<std::mutex> lock(command_receive_mutex);
            trajectory_tracking_statistics_update(t_now);

            // Run controller
            mpcController.update(
                t_now, m_vehicleState, m_vehicleCommandTrajectory,
                motor_throttle, steering_servo);
            //trajectory_controller_linear(t_now, motor_throttle, steering_servo);
        }
        break;

        default: // Direct
        {
            motor_throttle = m_vehicleCommandDirect.motor_throttle();
            steering_servo = m_vehicleCommandDirect.steering_servo();
        }
        break;
    }
    // cpm::TimeMeasurement::Instance().stop("mpc_switch");

    motor_throttle = fmax(-1.0, fmin(1.0, motor_throttle));
    steering_servo = fmax(-1.0, fmin(1.0, steering_servo));

    out_motor_throttle = motor_throttle;
    out_steering_servo = steering_servo;
}

void Controller::get_stop_signals(double &out_motor_throttle, double &out_steering_servo) 
{
    //Init. values
    double steering_servo = 0;
    double speed_target = 0;
    const double speed_measured = m_vehicleState.speed();
    
    // P controller to reach 0 speed (without "backshooting" like with the PI controller in speed controller)
    double motor_throttle = ((speed_target>=0)?(1.0):(-1.0)) * pow(fabs(0.152744 * speed_target),(0.627910));
    const double speed_error = speed_target - speed_measured;
    const double proportional_gain = 0.5; //We tested, and this seems to be an acceptable value (at least for central routing)
    motor_throttle += proportional_gain * speed_error;

    motor_throttle = fmax(-1.0, fmin(1.0, motor_throttle));
    steering_servo = fmax(-1.0, fmin(1.0, steering_servo));

    out_motor_throttle = motor_throttle;
    out_steering_servo = steering_servo;
}

void Controller::reset()
{
    //cpm::TimeMeasurement::Instance().start("reset_mutex");
    std::lock_guard<std::mutex> lock(command_receive_mutex);
    //cpm::TimeMeasurement::Instance().stop("reset_mutex");

    cpm::TimeMeasurement::Instance().start("reset_reader");
    reader_CommandDirect.reset(new cpm::Reader<VehicleCommandDirect>(topic_vehicleCommandDirect));
    reader_CommandSpeedCurvature.reset(new cpm::Reader<VehicleCommandSpeedCurvature>(topic_vehicleCommandSpeedCurvature));
    reader_CommandTrajectory.reset(new cpm::Reader<VehicleCommandTrajectory>(topic_vehicleCommandTrajectory));
    cpm::TimeMeasurement::Instance().stop("reset_reader");

    //Set current state to stop until new commands are received
    state = ControllerState::Stop;
}