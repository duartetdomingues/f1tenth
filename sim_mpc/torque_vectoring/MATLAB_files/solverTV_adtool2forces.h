#ifdef __cplusplus
extern "C" {
#endif
    
extern solver_int32_default solverTV_adtool2forces(solverTV_float *x,        /* primal vars                                         */
										 solverTV_float *y,        /* eq. constraint multiplers                           */
										 solverTV_float *l,        /* ineq. constraint multipliers                        */
										 solverTV_float *p,        /* parameters                                          */
										 solverTV_float *f,        /* objective function (scalar)                         */
										 solverTV_float *nabla_f,  /* gradient of objective function                      */
										 solverTV_float *c,        /* dynamics                                            */
										 solverTV_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
										 solverTV_float *h,        /* inequality constraints                              */
										 solverTV_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
										 solverTV_float *hess,     /* Hessian (column major)                              */
										 solver_int32_default stage,     /* stage number (0 indexed)                            */
										 solver_int32_default iteration, /* iteration number of solver                          */
										 solver_int32_default threadID  /* Id of caller thread */);

#ifdef __cplusplus
} /* extern "C" */
#endif
