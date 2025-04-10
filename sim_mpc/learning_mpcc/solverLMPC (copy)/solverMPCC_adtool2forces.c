/*
 * AD tool to FORCESPRO Template - missing information to be filled in by createADTool.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2023. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/solverMPCC.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif


#include "solverMPCC_casadi.h"



/* copies data from sparse matrix into a dense one */
static void solverMPCC_sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, solverMPCC_callback_float *data, solverMPCC_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((solverMPCC_float) data[j]);
        }
    }
}




/* AD tool to FORCESPRO interface */
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
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* AD tool input and output arrays */
    const solverMPCC_callback_float *in[4];
    solverMPCC_callback_float *out[7];
    

    /* Allocate working arrays for AD tool */
    
    solverMPCC_callback_float w[1206];
	
    /* temporary storage for AD tool sparse output */
    solverMPCC_callback_float this_f = (solverMPCC_callback_float) 0.0;
    solverMPCC_callback_float nabla_f_sparse[9];
    solverMPCC_callback_float h_sparse[2];
    solverMPCC_callback_float nabla_h_sparse[2];
    solverMPCC_callback_float c_sparse[14];
    solverMPCC_callback_float nabla_c_sparse[43];
            
    
    /* pointers to row and column info for 
     * column compressed format used by AD tool */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for AD tool */
	in[0] = x;
	in[1] = p;
	in[2] = l;
	in[3] = y;

	if ((stage >= 0) && (stage < 1))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		solverMPCC_objective_1(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = solverMPCC_objective_1_sparsity_out(1)[0];
			ncol = solverMPCC_objective_1_sparsity_out(1)[1];
			colind = solverMPCC_objective_1_sparsity_out(1) + 2;
			row = solverMPCC_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		solverMPCC_dynamics_1(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = solverMPCC_dynamics_1_sparsity_out(0)[0];
			ncol = solverMPCC_dynamics_1_sparsity_out(0)[1];
			colind = solverMPCC_dynamics_1_sparsity_out(0) + 2;
			row = solverMPCC_dynamics_1_sparsity_out(0) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = solverMPCC_dynamics_1_sparsity_out(1)[0];
			ncol = solverMPCC_dynamics_1_sparsity_out(1)[1];
			colind = solverMPCC_dynamics_1_sparsity_out(1) + 2;
			row = solverMPCC_dynamics_1_sparsity_out(1) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		solverMPCC_inequalities_1(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = solverMPCC_inequalities_1_sparsity_out(0)[0];
			ncol = solverMPCC_inequalities_1_sparsity_out(0)[1];
			colind = solverMPCC_inequalities_1_sparsity_out(0) + 2;
			row = solverMPCC_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = solverMPCC_inequalities_1_sparsity_out(1)[0];
			ncol = solverMPCC_inequalities_1_sparsity_out(1)[1];
			colind = solverMPCC_inequalities_1_sparsity_out(1) + 2;
			row = solverMPCC_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

	if ((stage >= 1) && (stage < 29))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		solverMPCC_objective_2(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = solverMPCC_objective_2_sparsity_out(1)[0];
			ncol = solverMPCC_objective_2_sparsity_out(1)[1];
			colind = solverMPCC_objective_2_sparsity_out(1) + 2;
			row = solverMPCC_objective_2_sparsity_out(1) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		solverMPCC_dynamics_2(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = solverMPCC_dynamics_2_sparsity_out(0)[0];
			ncol = solverMPCC_dynamics_2_sparsity_out(0)[1];
			colind = solverMPCC_dynamics_2_sparsity_out(0) + 2;
			row = solverMPCC_dynamics_2_sparsity_out(0) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = solverMPCC_dynamics_2_sparsity_out(1)[0];
			ncol = solverMPCC_dynamics_2_sparsity_out(1)[1];
			colind = solverMPCC_dynamics_2_sparsity_out(1) + 2;
			row = solverMPCC_dynamics_2_sparsity_out(1) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		solverMPCC_inequalities_2(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = solverMPCC_inequalities_2_sparsity_out(0)[0];
			ncol = solverMPCC_inequalities_2_sparsity_out(0)[1];
			colind = solverMPCC_inequalities_2_sparsity_out(0) + 2;
			row = solverMPCC_inequalities_2_sparsity_out(0) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = solverMPCC_inequalities_2_sparsity_out(1)[0];
			ncol = solverMPCC_inequalities_2_sparsity_out(1)[1];
			colind = solverMPCC_inequalities_2_sparsity_out(1) + 2;
			row = solverMPCC_inequalities_2_sparsity_out(1) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

	if ((stage >= 29) && (stage < 30))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		solverMPCC_objective_30(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = solverMPCC_objective_30_sparsity_out(1)[0];
			ncol = solverMPCC_objective_30_sparsity_out(1)[1];
			colind = solverMPCC_objective_30_sparsity_out(1) + 2;
			row = solverMPCC_objective_30_sparsity_out(1) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		solverMPCC_dynamics_30(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = solverMPCC_dynamics_30_sparsity_out(0)[0];
			ncol = solverMPCC_dynamics_30_sparsity_out(0)[1];
			colind = solverMPCC_dynamics_30_sparsity_out(0) + 2;
			row = solverMPCC_dynamics_30_sparsity_out(0) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = solverMPCC_dynamics_30_sparsity_out(1)[0];
			ncol = solverMPCC_dynamics_30_sparsity_out(1)[1];
			colind = solverMPCC_dynamics_30_sparsity_out(1) + 2;
			row = solverMPCC_dynamics_30_sparsity_out(1) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

	}

	if ((stage >= 30) && (stage < 31))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		solverMPCC_objective_31(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = solverMPCC_objective_31_sparsity_out(1)[0];
			ncol = solverMPCC_objective_31_sparsity_out(1)[1];
			colind = solverMPCC_objective_31_sparsity_out(1) + 2;
			row = solverMPCC_objective_31_sparsity_out(1) + 2 + (ncol + 1);
			solverMPCC_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

	}


    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((solverMPCC_float) this_f);
    }

    return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
