#include "MpcController.hpp"
#include <cassert>
#include <iostream>



static inline uint64_t get_time_ns()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}


MpcController::MpcController()
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

        casadi_vars[name] = std::vector<casadi_real>(n_rows * n_cols, 0.0);
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




    // temp test, delete later
    auto t_start = get_time_ns();

    for (int i = 0; i < 50; ++i)
    {

        // tmp, test eval
        casadi_mpc_fn(
            (const casadi_real**)(casadi_arguments.data()), 
            casadi_results.data(), 
            nullptr, nullptr, nullptr);
    }



    auto t_end = get_time_ns();

    std::cout << "dt " << (t_end - t_start) << std::endl;

}



void MpcController::update(
    uint64_t t_now, 
    const VehicleState &vehicleState,
    const std::map<uint64_t, TrajectoryPoint> &trajectory_points,
    double &motor_throttle, 
    double &steering_servo
)
{
    battery_voltage_lowpass_filtered += 0.1 * (vehicleState.battery_voltage() - battery_voltage_lowpass_filtered);

    const VehicleState vehicleState_predicted_start = delay_compensation_prediction(vehicleState);


    // TODO reference trajectory interpolation


    // TODO run MPC optimization


    // TODO shift output history, save new output
}



VehicleState MpcController::delay_compensation_prediction(
    const VehicleState &vehicleState
)
{
    const double dt_control_loop = 0.02;

    double px = vehicleState.pose().x();
    double py = vehicleState.pose().y();
    double yaw = vehicleState.pose().yaw();
    double speed = vehicleState.speed();


    for (int i = 0; i < MPC_DELAY_COMPENSATION_STEPS; ++i)
    {
        const auto &p = dynamics_parameters;
        const double delta = steering_output_history[i] + p[9-1];
        const double f = motor_output_history[i];
        const double d_px = p[1-1] * speed * (1 + p[2-1] * delta*delta) * cos(yaw + p[3-1] * delta + p[10-1]);
        const double d_py = p[1-1] * speed * (1 + p[2-1] * delta*delta) * sin(yaw + p[3-1] * delta + p[10-1]);
        const double d_yaw = p[4-1] * speed * delta;
        const double d_speed = p[5-1] * speed + (p[6-1] + p[7-1] * battery_voltage_lowpass_filtered) * ((f>=0)?(1.0):(-1.0)) * pow(fabs(f), p[8-1]);

        px += dt_control_loop * d_px;
        py += dt_control_loop * d_py;
        yaw += dt_control_loop * d_yaw;
        speed += dt_control_loop * d_speed;
    }

    VehicleState vehicleState_predicted_start = vehicleState;
    vehicleState_predicted_start.pose().x(px);
    vehicleState_predicted_start.pose().y(py);
    vehicleState_predicted_start.pose().yaw(yaw);
    vehicleState_predicted_start.speed(speed);
    return vehicleState_predicted_start;
}