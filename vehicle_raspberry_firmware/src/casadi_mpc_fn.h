/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int casadi_mpc_fn(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem);
void casadi_mpc_fn_incref(void);
void casadi_mpc_fn_decref(void);
casadi_int casadi_mpc_fn_n_out(void);
casadi_int casadi_mpc_fn_n_in(void);
const char* casadi_mpc_fn_name_in(casadi_int i);
const char* casadi_mpc_fn_name_out(casadi_int i);
const casadi_int* casadi_mpc_fn_sparsity_in(casadi_int i);
const casadi_int* casadi_mpc_fn_sparsity_out(casadi_int i);
int casadi_mpc_fn_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#ifdef __cplusplus
} /* extern "C" */
#endif
