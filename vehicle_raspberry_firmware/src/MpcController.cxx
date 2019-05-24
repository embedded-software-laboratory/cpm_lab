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