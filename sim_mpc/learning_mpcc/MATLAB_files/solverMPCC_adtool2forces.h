#ifdef __cplusplus
extern "C" {
#endif
    
extern solver_int32_default solverMPCC_adtool2forces(solverMPCC_float *x,        /* primal vars                                         */
										 solverMPCC_float *y,        /* eq. constraint multiplers                           */
										 solverMPCC_float *l,        /* ineq. constraint multipliers                        */
										 solverMPCC_float *p,        /* parameters                                          */
										 solverMPCC_float *f,        /* objective function (scalar)                         */
										 solverMPCC_float *nabla_f,  /* gradient of objective function                      */
										 solverMPCC_float *c,        /* dynamics                                            */
										 solverMPCC_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
										 solverMPCC_float *h,        /* inequality constraints                              */
										 solverMPCC_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
										 solverMPCC_float *hess,     /* Hessian (column major)                              */
										 solver_int32_default stage,     /* stage number (0 indexed)                            */
										 solver_int32_default iteration, /* iteration number of solver                          */
										 solver_int32_default threadID  /* Id of caller thread */);

#ifdef __cplusplus
} /* extern "C" */
#endif
