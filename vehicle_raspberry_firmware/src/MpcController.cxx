#include "MpcController.hpp"
#include "casadi_mpc_fn.h"
#include <cassert>
#include <iostream>


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
    }
}