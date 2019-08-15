#include "MpcController.hpp"
#include <cassert>
#include <iostream>
#include <sstream>
#include "cpm/Logging.hpp"
#include "TrajectoryInterpolation.hpp"

MpcController::MpcController(uint8_t _vehicle_id)
:topic_Visualization(cpm::get_topic<Visualization>("visualization"))
,writer_Visualization
(
    dds::pub::DataWriter<Visualization>
    (
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        topic_Visualization
    )
)
,vehicle_id(_vehicle_id)
{
    const casadi_int n_in = casadi_mpc_fn_n_in();
    const casadi_int n_out = casadi_mpc_fn_n_out();

    // For each casadi input or output variable
    for (casadi_int i_var = 0; i_var < n_in + n_out; ++i_var)
    {
        const casadi_int* sparsity = nullptr;
        if (i_var < n_in) 
        {
            sparsity = casadi_mpc_fn_sparsity_in(i_var);
        } 
        else
        {
            sparsity = casadi_mpc_fn_sparsity_out(i_var - n_in);
        }
        assert(sparsity);

        const casadi_int n_rows = sparsity[0];
        const casadi_int n_cols = sparsity[1];

        // Check that the variable is in the sparse CCS form.
        // See also https://web.casadi.org/docs/#api-of-the-generated-code
        assert(sparsity[2] == 0);

        // Check that the variable is actually dense. 
        // This way we dont have to implement the general case CCS matrix access.
        for (int i_col = 0; i_col <= n_cols; ++i_col)
        {
            assert(sparsity[2 + i_col] == i_col * n_rows);

            if(i_col < n_cols)
            {
                for (int i_row = 0; i_row < n_rows; ++i_row)
                {
                    assert(sparsity[2 + n_cols + 1 + n_rows * i_col + i_row] == i_row);
                }
            }
        }

        // Generate buffers for casadi variables
        std::string name = "";
        if (i_var < n_in) 
        {
            name = casadi_mpc_fn_name_in(i_var);
        } 
        else
        {
            name = casadi_mpc_fn_name_out(i_var - n_in);
        }

        assert(casadi_vars.count(name) == 0);
        casadi_vars[name] = std::vector<casadi_real>(n_rows * n_cols, 1e-12); // Can not be zero exactly, because the CadADi gradient is wrong at zero
        casadi_vars_size[name] = std::array<casadi_int, 2>{n_rows, n_cols};
        casadi_real* p_buffer = casadi_vars[name].data();

        if (i_var < n_in) 
        {
            casadi_arguments.push_back(p_buffer);
        } 
        else
        {
            casadi_results.push_back(p_buffer);
        }
    }

    assert((casadi_int)(casadi_arguments.size()) == n_in);
    assert((casadi_int)(casadi_results.size()) == n_out);

    // Check work space size
    casadi_int sz_arg;
    casadi_int sz_res;
    casadi_int sz_iw;
    casadi_int sz_w;

    // The documentation does not make it clear in which case these values
    // would be different. Do more research if this assertion ever fails.
    assert(casadi_mpc_fn_work(&sz_arg, &sz_res, &sz_iw, &sz_w) == 0);
    assert(sz_arg == n_in);
    assert(sz_res == n_out);
    assert(sz_iw == 0);
    assert(sz_w == 0);

    // Check casadi sizes against expected values
    assert(casadi_vars_size["var_x0"][0] == 1);
    assert(casadi_vars_size["var_x0"][1] == 4);

    assert(casadi_vars_size["var_u0"][0] == 1);
    assert(casadi_vars_size["var_u0"][1] == 2);

    assert(casadi_vars_size["var_u"][0] == MPC_control_steps);
    assert(casadi_vars_size["var_u"][1] == 3);

    assert(casadi_vars_size["var_momentum"][0] == MPC_control_steps);
    assert(casadi_vars_size["var_momentum"][1] == 3);

    assert(casadi_vars_size["var_params"][0] == 10);
    assert(casadi_vars_size["var_params"][1] == 1);

    assert(casadi_vars_size["var_reference_trajectory_x"][0] == MPC_prediction_steps);
    assert(casadi_vars_size["var_reference_trajectory_x"][1] == 1);

    assert(casadi_vars_size["var_reference_trajectory_y"][0] == MPC_prediction_steps);
    assert(casadi_vars_size["var_reference_trajectory_y"][1] == 1);

    assert(casadi_vars_size["var_learning_rate"][0] == 1);
    assert(casadi_vars_size["var_learning_rate"][1] == 1);

    assert(casadi_vars_size["var_momentum_rate"][0] == 1);
    assert(casadi_vars_size["var_momentum_rate"][1] == 1);

    assert(casadi_vars_size["trajectory_x"][0] == MPC_prediction_steps);
    assert(casadi_vars_size["trajectory_x"][1] == 1);

    assert(casadi_vars_size["trajectory_y"][0] == MPC_prediction_steps);
    assert(casadi_vars_size["trajectory_y"][1] == 1);

    assert(casadi_vars_size["objective"][0] == 1);
    assert(casadi_vars_size["objective"][1] == 1);

    assert(casadi_vars_size["var_momentum_next"][0] == MPC_control_steps);
    assert(casadi_vars_size["var_momentum_next"][1] == 3);

    assert(casadi_vars_size["var_u_next"][0] == MPC_control_steps);
    assert(casadi_vars_size["var_u_next"][1] == 3);
}

void MpcController::update(
    uint64_t t_now, 
    const VehicleState &vehicleState,
    const std::map<uint64_t, TrajectoryPoint> &trajectory_points,
    double &out_motor_throttle, 
    double &out_steering_servo
)
{
    battery_voltage_lowpass_filtered += 0.1 * (vehicleState.battery_voltage() - battery_voltage_lowpass_filtered);

    const VehicleState vehicleState_predicted_start = delay_compensation_prediction(vehicleState);

    std::vector<double> mpc_reference_trajectory_x;
    std::vector<double> mpc_reference_trajectory_y;

    if(!interpolate_reference_trajectory(
        t_now, 
        trajectory_points,
        mpc_reference_trajectory_x,
        mpc_reference_trajectory_y
    ))
    {
        reset_optimizer();
        out_motor_throttle = 0;
        out_steering_servo = 0;
        return;
    }

    assert(mpc_reference_trajectory_x.size() == MPC_prediction_steps);
    assert(mpc_reference_trajectory_y.size() == MPC_prediction_steps);

    optimize_control_inputs(
        vehicleState_predicted_start,
        mpc_reference_trajectory_x,
        mpc_reference_trajectory_y,
        out_motor_throttle, 
        out_steering_servo
    );

    // shift output history, save new output
    for (int i = 1; i < MPC_DELAY_COMPENSATION_STEPS; ++i)
    {
        motor_output_history[i-1] = motor_output_history[i];
        steering_output_history[i-1] = steering_output_history[i];
    }
    
    motor_output_history[MPC_DELAY_COMPENSATION_STEPS-1] = out_motor_throttle;
    steering_output_history[MPC_DELAY_COMPENSATION_STEPS-1] = out_steering_servo;
}


void MpcController::optimize_control_inputs(
    const VehicleState &vehicleState_predicted_start,
    const std::vector<double> &mpc_reference_trajectory_x,
    const std::vector<double> &mpc_reference_trajectory_y,
    double &out_motor_throttle, 
    double &out_steering_servo
)
{
    for (int i = 0; i < 20; ++i)
    {
        casadi_vars["var_x0"][0] = vehicleState_predicted_start.pose().x();
        casadi_vars["var_x0"][1] = vehicleState_predicted_start.pose().y();
        casadi_vars["var_x0"][2] = vehicleState_predicted_start.pose().yaw();
        casadi_vars["var_x0"][3] = vehicleState_predicted_start.speed();

        casadi_vars["var_u0"][0] = motor_output_history[MPC_DELAY_COMPENSATION_STEPS-1];
        casadi_vars["var_u0"][1] = steering_output_history[MPC_DELAY_COMPENSATION_STEPS-1];

        for (size_t j = 0; j < 2 * MPC_control_steps; ++j)
        {
            casadi_vars["var_u"][j] = fmin(1.0,fmax(-1.0,casadi_vars["var_u_next"][j]));
            casadi_vars["var_momentum"][j] = casadi_vars["var_momentum_next"][j];
        }

        // overwrite voltage, it is a measured disturbance, not an actual input
        for (size_t j = 0; j < MPC_control_steps; ++j)
        {
            casadi_vars["var_u"].at(2*MPC_control_steps + j) = battery_voltage_lowpass_filtered; 
            casadi_vars["var_momentum"].at(2*MPC_control_steps + j) = 0;
        }

        for (size_t j = 0; j < dynamics_parameters.size(); ++j)
        {
            casadi_vars["var_params"].at(j) = dynamics_parameters.at(j);
        }

        for (size_t j = 0; j < MPC_prediction_steps; ++j)
        {
            casadi_vars["var_reference_trajectory_x"][j] = mpc_reference_trajectory_x[j];
            casadi_vars["var_reference_trajectory_y"][j] = mpc_reference_trajectory_y[j];
        }

        casadi_vars["var_learning_rate"][0] = 0.4;
        casadi_vars["var_momentum_rate"][0] = 0.6;
        
        // Run casadi
        casadi_mpc_fn(
            (const casadi_real**)(casadi_arguments.data()), 
            casadi_results.data(), 
            nullptr, nullptr, nullptr);

    }

    //cpm::Logging::Instance().write("objective value %f ",casadi_vars["objective"][0]);

    if(casadi_vars["objective"][0] < 0.7)
    {
        out_motor_throttle = fmin(1.0,fmax(-1.0,casadi_vars["var_u_next"][0]));
        out_steering_servo = fmin(1.0,fmax(-1.0,casadi_vars["var_u_next"][MPC_control_steps]));


        // publish visualization of predicted trajectory
        Visualization vis;
        vis.id("mpc_prediction_" + std::to_string(vehicle_id));
        vis.type(VisualizationType::LineStrips);
        vis.time_to_live(2000000000ull);
        vis.size(0.12);
        vis.color().r(255);
        vis.color().g(0);
        vis.color().b(240);
        vis.points().resize(MPC_prediction_steps);
        for (size_t j = 0; j < MPC_prediction_steps; ++j)
        {
            vis.points().at(j).x(casadi_vars["trajectory_x"][j]);
            vis.points().at(j).y(casadi_vars["trajectory_y"][j]);
        }
        writer_Visualization.write(vis);


        /*
        std::ostringstream oss;

        oss << "ref_x = [";
        for (size_t j = 0; j < MPC_prediction_steps; ++j)
        {
            if(j>0) oss << ",";
            oss << mpc_reference_trajectory_x[j];
        }
        oss << "];";

        oss << "ref_y = [";
        for (size_t j = 0; j < MPC_prediction_steps; ++j)
        {
            if(j>0) oss << ",";
            oss << mpc_reference_trajectory_y[j];
        }
        oss << "];";

        oss << "pred_x = [";
        for (size_t j = 0; j < MPC_prediction_steps; ++j)
        {
            if(j>0) oss << ",";
            oss << casadi_vars["trajectory_x"][j];
        }
        oss << "];";

        oss << "pred_y = [";
        for (size_t j = 0; j < MPC_prediction_steps; ++j)
        {
            if(j>0) oss << ",";
            oss << casadi_vars["trajectory_y"][j];
        }
        oss << "];";


        std::string mpc_dbg = oss.str();
        std::cerr << mpc_dbg << std::endl;
        */

    }
    else
    {
        cpm::Logging::Instance().write(
            "Warning: Trajectory Controller: "
            "Large MPC objective %f. Provide a better reference trajectory. Stopping.", casadi_vars["objective"][0]);

        reset_optimizer();
        out_motor_throttle = 0.0;
        out_steering_servo = 0.0;
    }
}

void MpcController::reset_optimizer()
{
    for(auto &casadi_var:casadi_vars)
    {
        for (size_t i = 0; i < casadi_var.second.size(); ++i)
        {
            casadi_vars[casadi_var.first][i] = 1e-12; // Can not be zero exactly, because the CadADi gradient is wrong at zero
        }
    }
}



bool MpcController::interpolate_reference_trajectory(
    uint64_t t_now, 
    const std::map<uint64_t, TrajectoryPoint> &trajectory_points,
    std::vector<double> &out_mpc_reference_trajectory_x,
    std::vector<double> &out_mpc_reference_trajectory_y
)
{
    if(trajectory_points.size() < 2)
    {
        return false;
    }

    // interval of the MPC prediction
    const uint64_t t_start = t_now + MPC_DELAY_COMPENSATION_STEPS * (dt_control_loop * 1e9) + (dt_MPC * 1e9);
    const uint64_t t_end = t_start + (MPC_prediction_steps-1) * (dt_MPC * 1e9);

    // interval of the trajectory command
    const uint64_t t_trajectory_min = trajectory_points.begin()->first;
    const uint64_t t_trajectory_max = trajectory_points.rbegin()->first;


    if(t_trajectory_min >= t_start)
    {
        cpm::Logging::Instance().write(
            "Warning: Trajectory Controller: "
            "The trajectory command starts in the future.");
        return false;
    }

    if(t_trajectory_max < t_end)
    {
        cpm::Logging::Instance().write(
            "Warning: Trajectory Controller: "
            "The trajectory command has insufficient lead time. "
            "Increase lead time by %.2f ms.",
            double(t_end - t_trajectory_max) * 1e-6);
        return false;
    }

    out_mpc_reference_trajectory_x.resize(MPC_prediction_steps, 0);
    out_mpc_reference_trajectory_y.resize(MPC_prediction_steps, 0);

    for (size_t i = 0; i < MPC_prediction_steps; ++i)
    {
        const uint64_t t_interpolation = t_start + i * (dt_MPC * 1e9);


        const auto iterator_segment_end = trajectory_points.lower_bound(t_interpolation);

        assert(iterator_segment_end != trajectory_points.end());
        assert(iterator_segment_end != trajectory_points.begin());

        auto iterator_segment_start = iterator_segment_end;
        iterator_segment_start--;

        const TrajectoryPoint start_point = (*iterator_segment_start).second;
        const TrajectoryPoint end_point = (*iterator_segment_end).second;

        assert(t_interpolation >= start_point.t().nanoseconds());
        assert(t_interpolation <= end_point.t().nanoseconds());

        TrajectoryInterpolation trajectory_interpolation(t_interpolation, start_point, end_point);

        if(fabs(trajectory_interpolation.acceleration_x) > 10.0)
        {
            cpm::Logging::Instance().write(
                "Warning: Trajectory Controller: "
                "Large acceleration in reference trajectory. "
                "acceleration_x = %f",
                trajectory_interpolation.acceleration_x);
            return false;
        }

        if(fabs(trajectory_interpolation.acceleration_y) > 10.0)
        {
            cpm::Logging::Instance().write(
                "Warning: Trajectory Controller: "
                "Large acceleration in reference trajectory. "
                "acceleration_y = %f",
                trajectory_interpolation.acceleration_y);
            return false;
        }

        if(fabs(trajectory_interpolation.curvature) > 4.0)
        {
            cpm::Logging::Instance().write(
                "Warning: Trajectory Controller: "
                "Large curvature in reference trajectory. "
                "curvature = %f",
                trajectory_interpolation.curvature);
            return false;
        }

        out_mpc_reference_trajectory_x[i] = trajectory_interpolation.position_x;
        out_mpc_reference_trajectory_y[i] = trajectory_interpolation.position_y;
    }

    return true;
}


VehicleState MpcController::delay_compensation_prediction(
    const VehicleState &vehicleState
)
{
    double px = vehicleState.pose().x();
    double py = vehicleState.pose().y();
    double yaw = vehicleState.pose().yaw();
    double speed = vehicleState.speed();

    for (int i = 0; i < MPC_DELAY_COMPENSATION_STEPS; ++i)
    {
        VehicleModel::step(
            dynamics_parameters,
            dt_control_loop,
            motor_output_history[i],
            steering_output_history[i],
            battery_voltage_lowpass_filtered,
            px, py, yaw, speed
        );
    }

    VehicleState vehicleState_predicted_start = vehicleState;
    vehicleState_predicted_start.pose().x(px);
    vehicleState_predicted_start.pose().y(py);
    vehicleState_predicted_start.pose().yaw(yaw);
    vehicleState_predicted_start.speed(speed);
    return vehicleState_predicted_start;
}