#
# Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
# Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
# Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
# Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#
# author: Daniel Kloeser

import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, ocp_get_default_cmake_builder
from bicycle_model import bicycle_model
from typing import Tuple
from utils.indicies import StateIndex
from pathlib import Path


def acados_settings(
    s0, kapparef, d_left, d_right, stmpc_config, car_config, tire_config
) -> Tuple:
    # create render arguments
    ocp = AcadosOcp()

    # export model
    model, constraint, params = bicycle_model(
        s0, kapparef, d_left, d_right, stmpc_config, car_config, tire_config
    )

    # define acados ODE
    model_ac = AcadosModel()
    model_ac.f_expl_expr = model.f_expl_expr
    # define constraint：[longitudinal force, lateral force, inner_bound, outer_bound]
    model_ac.x = model.x
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    ocp.model = model_ac

    # Set parameters dimension
    p = get_parameters(stmpc_config)
    params.p = p
    ocp.parameter_values = p

    # dimensions
    nx = model.n_x
    nu = model.n_u
    ny = nx + nu
    ny_e = nx
    
    #State and input constraint
    input_constraint = False
    vx_max_constraint = True
    
    # Nonlinear constraint
    boundaries_constraint = True # if use boundary constraint for nonlinear constraint
    alat_constraint = False # if use throttle and delta constraint for state constraint
    soft = True # if use soft constraint for nonlinear constraint

    
    # define the number of soft constraints
    nsbx = 0  # state soft constraint
    nsbu = 0
    nsh = constraint.expr.shape[0]
    nsh_e = constraint.expr_e.shape[0]
    ns = nsh + nsbx + nsbu

    ocp.dims.nx = nx
    ocp.dims.nu = nu
    ocp.dims.ny = 0
    ocp.dims.ny_e = 0
    if soft:
        ocp.dims.nsh = nsh
        ocp.dims.nsh_e = nsh_e
        ocp.dims.ns = ns

    # use external costfunction defined in bicycle.py
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    ocp.model.cost_expr_ext_cost = model.cost_expr_ext_cost
    ocp.model.cost_expr_ext_cost_e = model.cost_expr_ext_cost_e

    # slack variables included state, input and nonlinear constraints
    # minmum  slack variables: sl
    # maximum slack varibales: su


    if soft:
        # Slack penalty function is Z_l*s_l**2 + Z_u*s_u**2
        # at stage 0
        ocp.cost.Zl_0 = stmpc_config.Zl * np.ones((constraint.expr.shape[0],))
        ocp.cost.Zu_0 = stmpc_config.Zu * np.ones((constraint.expr.shape[0],))
        # must have in the cost function, but not used (quadratic cost is enough to handle the constraints)
        ocp.cost.zl_0 = stmpc_config.zl * np.ones((constraint.expr.shape[0],))
        ocp.cost.zu_0 = stmpc_config.zu * np.ones((constraint.expr.shape[0],))
        # from 1 to N-1
        ocp.cost.Zl = stmpc_config.Zl * np.ones((constraint.expr.shape[0],))
        ocp.cost.Zu = stmpc_config.Zu * np.ones((constraint.expr.shape[0],))
        ocp.cost.zl = stmpc_config.zl * np.ones((constraint.expr.shape[0],))
        ocp.cost.zu = stmpc_config.zu * np.ones((constraint.expr.shape[0],))
        # at stage N
        ocp.cost.Zl_e = stmpc_config.Zl * np.ones((constraint.expr_e.shape[0],))
        ocp.cost.Zu_e = stmpc_config.Zu * np.ones((constraint.expr_e.shape[0],))
        ocp.cost.zl_e = stmpc_config.zl * np.ones((constraint.expr_e.shape[0],))
        ocp.cost.zu_e = stmpc_config.zu * np.ones((constraint.expr_e.shape[0],))

    # input constraint
    if input_constraint:
        ocp.constraints.lbu_0 = np.array([constraint.ddelta_min ,constraint.dthrottle_min])
        ocp.constraints.ubu_0 = np.array([constraint.ddelta_max, constraint.dthrottle_max])
        ocp.constraints.idxbu_0 = np.array([0, 1])

        ocp.constraints.lbu_e = np.array([constraint.ddelta_min ,constraint.dthrottle_min])
        ocp.constraints.ubu_e = np.array([constraint.ddelta_max ,constraint.dthrottle_max])
        ocp.constraints.idxbu_e = np.array([0, 1])

        ocp.constraints.lbu = np.array([constraint.ddelta_min ,constraint.dthrottle_min])
        ocp.constraints.ubu = np.array([constraint.ddelta_max, constraint.dthrottle_max])
        ocp.constraints.idxbu = np.array([0, 1])

    # state constraint
    
    if vx_max_constraint == True:
        # with acceleration constraint
        state_constraint_min = np.array(
            [constraint.v_x_min, constraint.delta_min, constraint.throttle_min]
        )
        state_constraint_max = np.array(
            [constraint.v_x_max, constraint.delta_max, constraint.throttle_max]
        )
        state_constraint_index = np.array(
            [
                StateIndex.VELOCITY_V_X.value,
                StateIndex.DELTA.value,
                StateIndex.THROTTLE.value,
            ]
        )
    else:
        # Remove acceleration constraint
        state_constraint_min = np.array(
            [constraint.delta_min, constraint.throttle_min]
        )
        state_constraint_max = np.array(
            [constraint.delta_max, constraint.throttle_max]
        )
        state_constraint_index = np.array(
            [
                StateIndex.DELTA.value,
                StateIndex.THROTTLE.value,
            ]
        )


    ocp.constraints.lbx_0 = state_constraint_min
    ocp.constraints.ubx_0 = state_constraint_max
    ocp.constraints.idxbx_0 = state_constraint_index

    ocp.constraints.lbx = state_constraint_min
    ocp.constraints.ubx = state_constraint_max
    ocp.constraints.idxbx = state_constraint_index

    ocp.constraints.lbx_e = state_constraint_min
    ocp.constraints.ubx_e = state_constraint_max
    ocp.constraints.idxbx_e = state_constraint_index

    # state soft constraint the acceleration does not affect the speed directly so need soft constraint
    # TODO: now can not turn online velocity constraints
    # ocp.constraints.lsbx = np.array([-2])
    # ocp.constraints.usbx = np.array([5])
    # ocp.constraints.idxsbx = np.array([3])
    # ocp.constraints.lsbx_e = np.array([0.1])
    # ocp.constraints.usbx_e = np.array([0.1])
    # ocp.constraints.idxsbx_e = np.array([3])

    # Nonlinear constraint: right and left boundary and lateral acceleration
    # Nonlinear constraint
    Nonlinear_constraint_min = np.array([0, -model.track_max, 0])
    Nonlinear_constraint_max = np.array([model.track_max, 0, 1])
    # Nonlinear_soft_min_value = np.zeros(nsh)
    # Nonlinear_soft_max_value = np.zeros(nsh) + np.array([0.1, 0.1, 0])
    Nonlinear_constraint_index = np.array([num for num in range(nsh)])
    #Nonlinear_constraint_index = [] # do not use soft constraint for now
    
    n_constrain = 0
    n_constrain_e = 0
    
    if boundaries_constraint:
        n_constrain += 2
        n_constrain_e += 2

    if alat_constraint:
        n_constrain += 1
        
        
    if soft:
        ocp.constraints.idxsh_0 = Nonlinear_constraint_index[:n_constrain]
        ocp.constraints.idxsh = Nonlinear_constraint_index[:n_constrain]
        ocp.constraints.idxsh_e = Nonlinear_constraint_index[:n_constrain_e]

   

    # at stage 0
    ocp.constraints.lh_0 = Nonlinear_constraint_min[:n_constrain]
    ocp.constraints.uh_0 = Nonlinear_constraint_max[:n_constrain]
    # ocp.constraints.lsh_0 = Nonlinear_soft_min_value
    # ocp.constraints.ush_0 = Nonlinear_soft_max_value

    # from stage 1 to N-1
    ocp.constraints.lh = Nonlinear_constraint_min[:n_constrain]
    ocp.constraints.uh = Nonlinear_constraint_max[:n_constrain]
    # ocp.constraints.lsh = Nonlinear_soft_min_value
    # ocp.constraints.ush = Nonlinear_soft_max_value

    # at stage N
    ocp.constraints.lh_e = Nonlinear_constraint_min[:n_constrain_e]
    ocp.constraints.uh_e = Nonlinear_constraint_max[:n_constrain_e]
    # ocp.constraints.lsh_e = Nonlinear_soft_min_value
    # ocp.constraints.ush_e = Nonlinear_soft_max_value

    ocp.model.con_h_expr_0 = constraint.expr[:n_constrain]
    ocp.model.con_h_expr = constraint.expr[:n_constrain]
    ocp.model.con_h_expr_e = constraint.expr_e[:n_constrain_e]

    # set to make intial condition necessary (otherwise lbx_0 owerwrites this)
    ocp.constraints.x0 = model.x0

    # discretization
    ocp.solver_options.N_horizon = stmpc_config.N  # Use AcadosOcpOptions.N_horizon for future compatibility
    # set QP solver and integration
    ocp.solver_options.tf = stmpc_config.N / stmpc_config.MPC_freq
    #ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    #ocp.solver_options.regularize_method = "MIRROR"
    #ocp.solver_options.globalization = "MERIT_BACKTRACKING"
    #ocp.solver_options.line_search_use_sufficient_descent = 1
    #ocp.solver_options.globalization_alpha_min = 1e-4
    #ocp.solver_options.alpha_reduction = 0.7
    #ocp.solver_options.nlp_solver_max_iter = 20   # default é 10
    #ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.hessian_approx = "EXACT"  # NOTE: do not believe the acados warning, setting hessian approximation to "EXACT" makes the solver fail
    #ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    # ocp.solver_options.ext_cost_num_hess = 1            # (opcional p/ robustez)
    #ocp.solver_options.levenberg_marquardt = 1e-2  # ~1e-6 a 1e-3
    """ ocp.solver_options.qpscaling_scale_constraints = "INF_NORM"
    ocp.solver_options.qpscaling_scale_objective   = "OBJECTIVE_GERSHGORIN" """

    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.tol = 1e-3
    ocp.solver_options.print_level = 0
    ocp.solver_options.qp_solver_warm_start = 1
    
    
    script_dir = Path(__file__).resolve().parent

    json_file = str(script_dir / "acados_ocp.json")
    
    print("JSON file generated in ", json_file)

    cmake = ocp_get_default_cmake_builder()
    
    ocp.code_export_directory = str(script_dir / "c_generated_code")
    
    print("CMakeLists.txt generated in ", ocp.code_export_directory)


    # create solver
    acados_solver = AcadosOcpSolver(ocp, json_file=json_file, cmake_builder=cmake, generate=True, build=True)

    return constraint, model, acados_solver, params


def get_parameters(cfg) -> np.ndarray:
    """Get parameters from stmpc_config"""
    params = np.array(
        [
            cfg.qs, cfg.qbeta, cfg.qddelta, cfg.qdthrottle, cfg.qvx, cfg.qvy, cfg.track_safety_margin
        ]
    ).astype(float)

    return params
