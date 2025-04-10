/*
 * AD tool to FORCESPRO Template - missing information to be filled in by createADTool.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2023. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif

#include "include/solverTV.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif


#include "solverTV_casadi.h"



/* copies data from sparse matrix into a dense one */
static void solverTV_sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, solverTV_callback_float *data, solverTV_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((solverTV_float) data[j]);
        }
    }
}




/* AD tool to FORCESPRO interface */
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
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
                                 solver_int32_default iteration, /* iteration number of solver                         */
                                 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* AD tool input and output arrays */
    const solverTV_callback_float *in[4];
    solverTV_callback_float *out[7];
    

    /* Allocate working arrays for AD tool */
    
    solverTV_callback_float w[30];
	
    /* temporary storage for AD tool sparse output */
    solverTV_callback_float this_f = (solverTV_callback_float) 0.0;
    solverTV_callback_float nabla_f_sparse[9];
    solverTV_callback_float h_sparse[10];
    solverTV_callback_float nabla_h_sparse[13];
    solverTV_callback_float c_sparse[4];
    solverTV_callback_float nabla_c_sparse[15];
    
    
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
		solverTV_objective_1(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = solverTV_objective_1_sparsity_out(1)[0];
			ncol = solverTV_objective_1_sparsity_out(1)[1];
			colind = solverTV_objective_1_sparsity_out(1) + 2;
			row = solverTV_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		solverTV_dynamics_1(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = solverTV_dynamics_1_sparsity_out(0)[0];
			ncol = solverTV_dynamics_1_sparsity_out(0)[1];
			colind = solverTV_dynamics_1_sparsity_out(0) + 2;
			row = solverTV_dynamics_1_sparsity_out(0) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = solverTV_dynamics_1_sparsity_out(1)[0];
			ncol = solverTV_dynamics_1_sparsity_out(1)[1];
			colind = solverTV_dynamics_1_sparsity_out(1) + 2;
			row = solverTV_dynamics_1_sparsity_out(1) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		solverTV_inequalities_1(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = solverTV_inequalities_1_sparsity_out(0)[0];
			ncol = solverTV_inequalities_1_sparsity_out(0)[1];
			colind = solverTV_inequalities_1_sparsity_out(0) + 2;
			row = solverTV_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = solverTV_inequalities_1_sparsity_out(1)[0];
			ncol = solverTV_inequalities_1_sparsity_out(1)[1];
			colind = solverTV_inequalities_1_sparsity_out(1) + 2;
			row = solverTV_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

	if ((stage >= 1) && (stage < 13))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		solverTV_objective_2(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = solverTV_objective_2_sparsity_out(1)[0];
			ncol = solverTV_objective_2_sparsity_out(1)[1];
			colind = solverTV_objective_2_sparsity_out(1) + 2;
			row = solverTV_objective_2_sparsity_out(1) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		solverTV_dynamics_2(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = solverTV_dynamics_2_sparsity_out(0)[0];
			ncol = solverTV_dynamics_2_sparsity_out(0)[1];
			colind = solverTV_dynamics_2_sparsity_out(0) + 2;
			row = solverTV_dynamics_2_sparsity_out(0) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = solverTV_dynamics_2_sparsity_out(1)[0];
			ncol = solverTV_dynamics_2_sparsity_out(1)[1];
			colind = solverTV_dynamics_2_sparsity_out(1) + 2;
			row = solverTV_dynamics_2_sparsity_out(1) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		solverTV_inequalities_2(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = solverTV_inequalities_2_sparsity_out(0)[0];
			ncol = solverTV_inequalities_2_sparsity_out(0)[1];
			colind = solverTV_inequalities_2_sparsity_out(0) + 2;
			row = solverTV_inequalities_2_sparsity_out(0) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = solverTV_inequalities_2_sparsity_out(1)[0];
			ncol = solverTV_inequalities_2_sparsity_out(1)[1];
			colind = solverTV_inequalities_2_sparsity_out(1) + 2;
			row = solverTV_inequalities_2_sparsity_out(1) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

	if ((stage >= 13) && (stage < 14))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		solverTV_objective_14(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = solverTV_objective_14_sparsity_out(1)[0];
			ncol = solverTV_objective_14_sparsity_out(1)[1];
			colind = solverTV_objective_14_sparsity_out(1) + 2;
			row = solverTV_objective_14_sparsity_out(1) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		solverTV_dynamics_14(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = solverTV_dynamics_14_sparsity_out(0)[0];
			ncol = solverTV_dynamics_14_sparsity_out(0)[1];
			colind = solverTV_dynamics_14_sparsity_out(0) + 2;
			row = solverTV_dynamics_14_sparsity_out(0) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = solverTV_dynamics_14_sparsity_out(1)[0];
			ncol = solverTV_dynamics_14_sparsity_out(1)[1];
			colind = solverTV_dynamics_14_sparsity_out(1) + 2;
			row = solverTV_dynamics_14_sparsity_out(1) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		solverTV_inequalities_14(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = solverTV_inequalities_14_sparsity_out(0)[0];
			ncol = solverTV_inequalities_14_sparsity_out(0)[1];
			colind = solverTV_inequalities_14_sparsity_out(0) + 2;
			row = solverTV_inequalities_14_sparsity_out(0) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = solverTV_inequalities_14_sparsity_out(1)[0];
			ncol = solverTV_inequalities_14_sparsity_out(1)[1];
			colind = solverTV_inequalities_14_sparsity_out(1) + 2;
			row = solverTV_inequalities_14_sparsity_out(1) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

	if ((stage >= 14) && (stage < 15))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		solverTV_objective_15(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = solverTV_objective_15_sparsity_out(1)[0];
			ncol = solverTV_objective_15_sparsity_out(1)[1];
			colind = solverTV_objective_15_sparsity_out(1) + 2;
			row = solverTV_objective_15_sparsity_out(1) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		solverTV_inequalities_15(in, out, NULL, w, 0);
		if (h != NULL)
		{
			nrow = solverTV_inequalities_15_sparsity_out(0)[0];
			ncol = solverTV_inequalities_15_sparsity_out(0)[1];
			colind = solverTV_inequalities_15_sparsity_out(0) + 2;
			row = solverTV_inequalities_15_sparsity_out(0) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if (nabla_h != NULL)
		{
			nrow = solverTV_inequalities_15_sparsity_out(1)[0];
			ncol = solverTV_inequalities_15_sparsity_out(1)[1];
			colind = solverTV_inequalities_15_sparsity_out(1) + 2;
			row = solverTV_inequalities_15_sparsity_out(1) + 2 + (ncol + 1);
			solverTV_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}


    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((solverTV_float) this_f);
    }

    return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
