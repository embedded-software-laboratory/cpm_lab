#include "ObstacleSimulation.hpp"

ObstacleSimulation::ObstacleSimulation(std::vector<CommonTrajectoryPoint> _trajectory, double _time_step_size, int _id, bool _simulated_time)
:
writer_vehicleState(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), cpm::get_topic<VehicleState>("vehicleState")),
trajectory(_trajectory),
simulated_time(_simulated_time)
{
    //Set up cpm values (cpm init has already been done before)
    node_id = "obstacle_simulation"; //Will probably not be used, as main already set LabControlCenter

    //Translate time distance to nanoseconds
    //We expect given step size to be defined in seconds
    time_step_size = static_cast<uint64_t>(_time_step_size * 1e9);
    obstacle_id = static_cast<uint8_t>(_id % 255);
    
    dt_nanos = 20000000ull;
    if (time_step_size < 20000000ull)
    {
        dt_nanos = time_step_size;
    }

    //We do not accept empty trajectories
    assert(trajectory.size() > 0);
}

ObstacleSimulation::~ObstacleSimulation()
{
    if (timer)
    {
        timer->stop();
    }
    timer.reset();
}

void ObstacleSimulation::start()
{
    //Create timer here, if we do it at reset we might accidentally receive stop signals in between (-> unusable then)
    timer = cpm::Timer::create(node_id, dt_nanos, 0, false, true, simulated_time);

    //Remember start time, so that we can check how much time has passed / which state to choose when
    start_time = timer->get_time();

    timer->start_async([=] (uint64_t t_now) {
        // if (t_now - start_time > trajectory.at(trajectory.size() - 1).time.get_mean())
        // {
        //     timer->stop();
        // }

        //We must be able to use time.value(), as it is a required field
        assert(trajectory.at(current_trajectory).time.has_value());

        while (t_now - start_time >= trajectory.at(current_trajectory).time.value().get_mean() * time_step_size && current_trajectory < trajectory.size() - 1)
        {
            ++current_trajectory;
        }

        //Create current state from current time, get current trajectory point - TODO: More than trivial point-hopping
        VehicleState state;
        state.vehicle_id(obstacle_id);
        state.IPS_update_age_nanoseconds(0);

        //Set header
        Header header;
        header.create_stamp(TimeStamp(t_now));
        header.valid_after_stamp(TimeStamp(t_now));
        state.header(header);

        //These values are set either by interpolation or using the last data point
        double x;
        double y;
        double yaw;
        //Interpolate or stay at start / end point
        if (trajectory.at(current_trajectory).time.value().get_mean() * time_step_size >= t_now - start_time && current_trajectory > 0)
        {
            //Interpolate
            interpolate_between(trajectory.at(current_trajectory - 1), trajectory.at(current_trajectory), t_now - start_time, x, y, yaw);
        }
        else
        {
            //Stay at final point
            x = trajectory.at(current_trajectory).position.first;
            y = trajectory.at(current_trajectory).position.second;
            yaw = trajectory.at(current_trajectory).orientation.value_or(0.0);
        }

        //Set pose
        Pose2D pose;
        pose.x(x);
        pose.y(y);
        pose.yaw(yaw);
        state.pose(pose);

        //Set velocity, if it exists
        if(trajectory.at(current_trajectory).velocity.has_value())
        {
            state.speed(trajectory.at(current_trajectory).velocity.value().get_mean());
        }

        writer_vehicleState.write(state);
    });
}

void ObstacleSimulation::interpolate_between(CommonTrajectoryPoint p1, CommonTrajectoryPoint p2, double current_time, double &x_interp, double &y_interp, double &yaw_interp)
{
    if (p1.velocity.has_value() && p2.velocity.has_value())
    {
        //Like Janis Interpolation, but with other datatypes, thus copied and modified
        const double t_start = p1.time.value().get_mean() * time_step_size / 1e9;
        const double t_end = p2.time.value().get_mean() * time_step_size / 1e9;
        double t_now = current_time / 1e9;

        const double delta_t = t_end - t_start;
        const double tau = (t_now - t_start) / delta_t;
        
        const double tau2 = tau * tau;
        const double tau3 = tau * tau2;
        
        const double position_start_x = p1.position.first;
        const double position_start_y = p1.position.second;
        const double position_end_x = p2.position.first;
        const double position_end_y = p2.position.second;

        //Get velocity x, y - expectation: orientation value gives velocity orientation, not position values
        double p1_velocity_x = std::cos(p1.orientation.value_or(0.0)) * p1.velocity.value().get_mean() / 10;
        double p1_velocity_y = std::sin(p1.orientation.value_or(0.0)) * p1.velocity.value().get_mean() / 10;
        double p2_velocity_x = std::cos(p2.orientation.value_or(0.0)) * p2.velocity.value().get_mean() / 10;
        double p2_velocity_y = std::sin(p2.orientation.value_or(0.0)) * p2.velocity.value().get_mean() / 10;

        if(p1.orientation.value_or(0.0) > 1.0)
        {
            std::cout << "<" << p1_velocity_x << "|" << tau << "|" << tau2 << std::endl;
        }
        
        const double velocity_start_x = p1_velocity_x * delta_t;
        const double velocity_start_y = p1_velocity_y * delta_t;
        const double velocity_end_x = p2_velocity_x * delta_t;
        const double velocity_end_y = p2_velocity_y * delta_t;
        
        
        // Hermite spline coefficients
        const double p0 = 2*tau3 - 3*tau2 + 1;
        const double m0 = tau3 - 2*tau2 + tau;
        const double p1 = -2*tau3 + 3*tau2;
        const double m1 = tau3 - tau2;
        
        // Hermite spline derivative coefficients
        const double dp0 = 6*tau2 - 6*tau;
        const double dm0 = 3*tau2 - 4*tau + 1;
        const double dp1 = -6*tau2 + 6*tau;
        const double dm1 = 3*tau2 - 2*tau;  
        
        x_interp     =  position_start_x *   p0 + velocity_start_x *   m0 + position_end_x *   p1 + velocity_end_x *   m1;
        y_interp     =  position_start_y *   p0 + velocity_start_y *   m0 + position_end_y *   p1 + velocity_end_y *   m1;
        double velocity_x     = (position_start_x *  dp0 + velocity_start_x *  dm0 + position_end_x *  dp1 + velocity_end_x *  dm1) / delta_t;
        double velocity_y     = (position_start_y *  dp0 + velocity_start_y *  dm0 + position_end_y *  dp1 + velocity_end_y *  dm1) / delta_t;
        yaw_interp = std::atan2(velocity_y, velocity_x);
    }
    else
    {
        //Fallback: Linear interpolation (if no velocity is set)
        double intermediate_frac = (current_time - (p1.time.value().get_mean() * time_step_size)) / ((p2.time.value().get_mean() - p1.time.value().get_mean()) * time_step_size);
        x_interp = (p2.position.first - p1.position.first) * intermediate_frac + p1.position.first;
        y_interp = (p2.position.second - p1.position.second) * intermediate_frac + p1.position.second;
        yaw_interp = (p2.orientation.value_or(0.0) - p1.orientation.value_or(0.0)) * intermediate_frac + p1.orientation.value_or(0.0);
    }
}

void ObstacleSimulation::reset()
{
    if (timer)
    {
        timer->stop();
    }
    timer.reset();

    current_trajectory = 0;
    start_time = 0;
}