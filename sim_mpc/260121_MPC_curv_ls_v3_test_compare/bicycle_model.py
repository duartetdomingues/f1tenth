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

import types

# author: Daniel Kloeser

import casadi as cs
import numpy as np
from casadi import MX, Function, cos, fmod, interpolant, sin, vertcat, sqrt


def bicycle_model(
    s0: list,
    kapparef: list,
    d_left: list,
    d_right: list,
    stmpc_config,
    car_config,
    tire_config,
):
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "mpc_model"

    length = len(s0)
    pathlength = s0[-1]
    # copy loop to beginning and end

    """ s0 = np.append(s0, [s0[length - 1] + s0[1:length]])
    s0 = np.append([s0[: length - 1] - s0[length - 1]], s0)
    kapparef = np.append(kapparef, kapparef[1:length])
    kapparef = np.append([kapparef[: length - 1] - kapparef[length - 1]], kapparef)

    d_left = np.append(d_left, d_left[1:length])
    d_left = np.append([d_left[: length - 1] - d_left[length - 1]], d_left)
    d_right = np.append(d_right, d_right[1:length])
    d_right = np.append([d_right[: length - 1] - d_right[length - 1]], d_right) """

    # compute spline interpolations
    kapparef_s = interpolant("kapparef_s", "bspline", [s0], kapparef)
    left_bound_s = interpolant("left_bound_s", "bspline", [s0], d_left)
    right_bound_s = interpolant("right_bound_s", "bspline", [s0], d_right)

    # CasADi Model
    # set up states & controls
    s = MX.sym("s")
    n = MX.sym("n")
    theta = MX.sym("theta")
    v_x = MX.sym("v_x")
    # v_y = MX.sym("v_y")
    yaw_rate = MX.sym("yaw_rate")
    delta = MX.sym("delta")
    #throttle = MX.sym("throttle")
    x = vertcat(s, n, theta, v_x, yaw_rate, delta)
    n_x = x.size()[0]

    # controls
    u_delta_dot = MX.sym("u_delta_dot")
    u_v_x_dot = MX.sym("u_v_x_dot")
    u = vertcat(u_delta_dot, u_v_x_dot)
    n_u = u.size()[0]

    # xdot
    s_dot = MX.sym("s_dot")
    n_dot = MX.sym("n_dot")
    theta_dot = MX.sym("theta_dot")
    v_x_dot = MX.sym("v_x_dot")
    # v_y_dot = MX.sym("v_y_dot")
    yaw_rate_dot = MX.sym("yaw_rate_dot")

    # algebraic variables
    z = vertcat([])

    # parameters
    freq = stmpc_config.MPC_freq

    lr = car_config.lr
    lf = car_config.lf
    m = car_config.m

    # Rotational inertia
    # Iz = 1 / 12 * mass * (car_length**2 + track_width**2)  # 0.5 are the car_length
    # Iz = car_length * mass * Wheelbase**2; % 0.3 are de mass disturbation
    Iz = car_config.Iz
    h_cg = car_config.h_cg

    friction_coeff = tire_config.friction_coeff
    Bf = tire_config.Bf
    Cf = tire_config.Cf
    Df = tire_config.Df
    Ef = tire_config.Ef
    Br = tire_config.Br
    Cr = tire_config.Cr
    Dr = tire_config.Dr
    Er = tire_config.Er
    g = 9.81

    # # online parameters
    weight_ds = MX.sym("weight_ds")
    weight_beta = MX.sym("weight_beta")
    weight_ddelta = MX.sym("weight_ddelta")
    weight_dthrottle = MX.sym("weight_dthrottle")
    weight_qvx = MX.sym("weight_qvx")
    v_x_ref = MX.sym("v_x_ref")
    safety_margin = MX.sym("safety_margin")
    p = vertcat(
        weight_ds,
        weight_beta,
        weight_ddelta,
        weight_dthrottle,
        weight_qvx,
        v_x_ref,
        safety_margin,
    )  # lateral acceleration constraints
    n_p = p.size()[0]

    # Parameters fixed
    # Constant motor Force-Cm= Fx/T : T=0.20 , acc_x= 5 m/s2 m=3 -> F=15N -> 75
    C_motor = 75

    # rolling resistance force F_rr - Desacelaration when stop Duty =
    # acc_x=1.5 m/s2 F= 4.5N
    F_rr = 4.5

    # calculate the index
    s_mod = fmod(s, pathlength)

    # motor force
    kapparef = kapparef_s(s_mod)
    left_bound = left_bound_s(s_mod)
    right_bound = right_bound_s(s_mod)

    # dynamics
    # den = 1 - kapparef * n
    # den_safe = cs.if_else(
    #     cs.fabs(den) < 0.2, 0.2 * cs.sign(den), den
    # )  # 0.2 é conservador
    # s_dot = ((v_x * cos(theta)) - (v_y * sin(theta))) / den_safe
    s_dot = ((v_x * cos(theta))) / (1 - kapparef * n)
    n_dot = v_x * sin(theta)
    theta_dot = yaw_rate - (kapparef * s_dot)
    v_x_dot = u_v_x_dot
    
    yaw_rate_dot = v_x * u_delta_dot / (lr + lf) # Assumed constant speed
    
    delta_dot = u_delta_dot
    #yaw_rate_dot = 1 /(lr + lf) *(u_v_x_dot * delta + v_x * u_delta_dot) # Variable speed
    
    
    
    #yaw_rate_dot = (1 / Iz) * ((-F_yr * lr) + (F_yf * lf) * cos(delta))
    # yaw_rate_dot = (1 / Iz) * ((-F_yr * lr) + (F_yf * lf) * cos(delta) + F_xf * sin(delta))

    # delta inputs for smoother control

    # continuous dynamics
    f_expl = vertcat(
        s_dot, n_dot, theta_dot, v_x_dot, yaw_rate_dot, delta_dot
    )
    f_expl_func = Function(
        "f_expl_func",
        [s, n, theta, v_x, yaw_rate, delta, delta_dot, v_x_dot, p],
        [f_expl],
    )

    # Define initial conditions
    model.x0 = np.zeros(n_x)
    # model.x0 = np.array([32.6, 0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0], dtype=float)
    
    s_dot_ref = 5

    terminal_multiplier = 1

    model.cost_y_expr = vertcat(
        n,
        theta,
        s_dot,
        yaw_rate,
        delta_dot,
        v_x_dot
    )
    model.cost_y_expr_0 = model.cost_y_expr
    model.cost_y_expr_e = vertcat(*[model.cost_y_expr[i] for i in range(model.cost_y_expr.size()[0] - 2)])

    model.yref = np.array([
        0.0,         # n_ref
        0.0,         # theta_ref
        3.2,      # v_x_ref
        0.0,         # yaw_rate_ref
        0.0,         # ddelta_ref
        0.0          # dv_x_ref
    ])
    model.yref_0 = model.yref.copy()
    model.yref_e = model.yref[:-2].copy()
    
    model.ny = model.cost_y_expr.size()[0]
    model.ny_0 = model.ny
    model.ny_e = model.cost_y_expr_e.size()[0]
    
    model.W = np.diag([
        20,      # penalização de n
        20,     # penalização de theta
        8,     # penalização de erro em v_x
        7,     # penalização de erro em yaw_rate
        10,      # penalização de ddelta
        1       # penalização de dv_x
    ])
    model.W_0 = model.W
    model.W_e = model.W[:-2, :-2] * terminal_multiplier
    
    # constraint on lateral errors
    n_right_bound = n + right_bound - safety_margin
    n_left_bound = n - left_bound + safety_margin

    # nonlinear constraints
    """ if stmpc_config.correct_v_y_dot:
        a_lat = v_y_dot
    else:
        a_lat = v_x * v_x * cs.tan(delta) / (lr + lf)
    constraint.pathlength = pathlength
    # TODO: introduce asymmetric constraints (now you can just over constrain breaking) with a_long_min
    if stmpc_config.combined_constraints == "ellipse":
        combined_constraints = (a_lat / a_lat_max) ** 2 + (v_x_dot / a_long_max) ** 2
    elif stmpc_config.combined_constraints == "diamond":
        combined_constraints = cs.fabs(a_lat / a_lat_max) + cs.fabs(
            v_x_dot / a_long_max
        )
    else:
        combined_constraints = 0 """

    # TODO fix the n constraint with a single constraint
    # NOTE: easiest when non-terminal constraints have the extra constraints
    # only at the end, otherwise need to change acados_settings.py
    # constraint.expr = vertcat(n_right_bound, n_left_bound, combined_constraints)
    constraint.expr = vertcat(n_right_bound, n_left_bound)

    constraint.expr_e = vertcat(n_right_bound, n_left_bound)

    # read the parameters range from the yaml file

    # state bounds
    constraint.delta_min = stmpc_config.delta_min  # minimum steering angle [rad]
    constraint.delta_max = stmpc_config.delta_max  # maximum steering angle [rad]
    constraint.v_x_min = stmpc_config.v_min  # minimum velocity [m/s]
    constraint.v_x_max = stmpc_config.v_max  # maximum velocity [m/s]


    # input bounds
    constraint.ddelta_min = stmpc_config.ddelta_min # minimum change rate of stering angle [rad/s]
    constraint.ddelta_max = stmpc_config.ddelta_max # maximum change rate of steering angle [rad/s]
    constraint.dvx_min = -1 #minimum acceleration [m/s^2]
    constraint.dvx_max = 1 #maximum acceleration [m/s^2]

    # Define model struct
    params = types.SimpleNamespace()
    # model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.n_x = n_x
    # model.xdot = xdot
    model.u = u
    model.n_u = n_u
    model.z = z
    model.p = p
    model.name = model_name
    model.params = params
    model.kappa = kapparef_s
    model.f_expl_func = f_expl_func
    model.left_bound_s = left_bound_s
    model.right_bound_s = right_bound_s
    model.track_max = stmpc_config.track_max_width
    # model.forces_func = forces_func
    # model.alpha_func = alpha_func
    # model.safety_width = stmpc_config.track_safety_margin
    return model, constraint, params
