function [constraint, model, acados_solver, params] = acados_settings(s0, kapparef, d_left, d_right, stmpc_config, car_config, tire_config)
% acados_settings.m
% Convertido do Python para MATLAB (CasADi + acados).
% Constrói o OCP, define constraints/custos/opções e cria o AcadosOcpSolver.

import casadi.*

% OCP e modelo
ocp = AcadosOcp();

[model, constraint, params] = bicycle_model(s0, kapparef, d_left, d_right, stmpc_config, car_config, tire_config);

% Wrapper do modelo para o acados
model_ac = AcadosModel();
model_ac.f_expl_expr  = model.f_expl_expr;
model_ac.con_h_expr_0 = constraint.expr;
model_ac.con_h_expr   = constraint.expr;
model_ac.con_h_expr_e = constraint.expr_e;
model_ac.x            = model.x;
model_ac.u            = model.u;
model_ac.z            = model.z;
model_ac.p            = model.p;
model_ac.name         = model.name;

ocp.model = model_ac;

% Parâmetros
p = get_parameters(stmpc_config);
params.p = p;
ocp.parameter_values = p;

% Dimensões
nx = model.n_x;
nu = model.n_u;
nsbx = 0; nsbu = 0;
nsh   = size(constraint.expr, 1);
nsh_e = size(constraint.expr_e, 1);
ns    = nsh + nsbx + nsbu;

ocp.dims.N   = stmpc_config.N;
ocp.dims.nx  = nx;
ocp.dims.nu  = nu;
ocp.dims.ny  = 0;
ocp.dims.ny_e= 0;

% Custo externo
ocp.cost.cost_type   = 'EXTERNAL';
ocp.cost.cost_type_e = 'EXTERNAL';
ocp.model.cost_expr_ext_cost   = model.cost_expr_ext_cost;
ocp.model.cost_expr_ext_cost_e = model.cost_expr_ext_cost_e;

% Penalizações de slack
ocp.cost.Zl_0 = stmpc_config.Zl * ones(nsh,1);
ocp.cost.Zu_0 = stmpc_config.Zu * ones(nsh,1);
ocp.cost.zl_0 = stmpc_config.zl * ones(nsh,1);
ocp.cost.zu_0 = stmpc_config.zu * ones(nsh,1);

ocp.cost.Zl = stmpc_config.Zl * ones(ns,1);
ocp.cost.Zu = stmpc_config.Zu * ones(ns,1);
ocp.cost.zl = stmpc_config.zl * ones(ns,1);
ocp.cost.zu = stmpc_config.zu * ones(ns,1);

ocp.cost.Zl_e = stmpc_config.Zl * ones(nsh_e,1);
ocp.cost.Zu_e = stmpc_config.Zu * ones(nsh_e,1);
ocp.cost.zl_e = stmpc_config.zl * ones(nsh_e,1);
ocp.cost.zu_e = stmpc_config.zu * ones(nsh_e,1);

% Limites de entrada (u = [jerk; ddelta])
% ocp.constraints.lbu_0   = [constraint.jerk_min;   constraint.ddelta_min];
% ocp.constraints.ubu_0   = [constraint.jerk_max;   constraint.ddelta_max];
% ocp.constraints.idxbu_0 = int32([0; 1]);

ocp.constraints.lbu     = [constraint.jerk_min;   constraint.ddelta_min];
ocp.constraints.ubu     = [constraint.jerk_max;   constraint.ddelta_max];
ocp.constraints.idxbu   = int32([0; 1]);

% ocp.constraints.lbu_e   = [constraint.jerk_min;   constraint.ddelta_min];
% ocp.constraints.ubu_e   = [constraint.jerk_max;   constraint.ddelta_max];
% ocp.constraints.idxbu_e = int32([0; 1]);

% Limites de estado (em 0-based, acados): x = [s n theta v_x v_y delta yaw_rate accel]
% indices -> v_x(3), delta(5), accel(7)
state_min  = [constraint.v_x_min; constraint.delta_min; constraint.a_min];
state_max  = [constraint.v_x_max; constraint.delta_max; constraint.a_max];
state_idx0 = [3; 5; 7];

ocp.constraints.lbx_0   = state_min;
ocp.constraints.ubx_0   = state_max;
ocp.constraints.idxbx_0 = state_idx0;

ocp.constraints.lbx     = state_min;
ocp.constraints.ubx     = state_max;
ocp.constraints.idxbx   = state_idx0;

ocp.constraints.lbx_e   = state_min;
ocp.constraints.ubx_e   = state_max;
ocp.constraints.idxbx_e = state_idx0;

% Constraints não lineares h(x,u) \in [lh, uh]
Nonlin_min = [0; -model.track_max; 0];
Nonlin_max = [model.track_max; 0; 1];
Nonlin_idx = int32((0:(nsh-1)).'); % 0..nsh-1

ocp.constraints.lh_0    = Nonlin_min;
ocp.constraints.uh_0    = Nonlin_max;
ocp.constraints.idxsh_0 = Nonlin_idx;

ocp.constraints.lh      = Nonlin_min;
ocp.constraints.uh      = Nonlin_max;
ocp.constraints.idxsh   = Nonlin_idx;

ocp.constraints.lh_e    = Nonlin_min(1:nsh_e);
ocp.constraints.uh_e    = Nonlin_max(1:nsh_e);
ocp.constraints.idxsh_e = Nonlin_idx(1:nsh_e);

% Condição inicial
ocp.constraints.x0 = model.x0;

% Opções do solver
ocp.solver_options.tf                    = stmpc_config.N / stmpc_config.MPC_freq;
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.nlp_solver_type       = 'SQP_RTI';
ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON'; % EXACT tende a falhar
ocp.solver_options.integrator_type       = 'ERK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps  = 3;
ocp.solver_options.qp_solver_tol_stat = 1e-2;
ocp.solver_options.qp_solver_tol_eq=1e-2;
ocp.solver_options.qp_solver_tol_ineq=1e-2;
ocp.solver_options.qp_solver_tol_comp=1e-2;
ocp.solver_options.print_level           = 0;


acados_solver = AcadosOcpSolver(ocp);

end

function params = get_parameters(cfg)
% get_parameters - mesma ordem do Python

% Parâmetros online (ordem igual à do acados_settings/get_parameters)
params = [ ...
    2;                                   % V_target (placeholder; podes ligar a um alvo real)
    cfg.qadv;                   % weight_adv
    cfg.qv;                     % weight_qv
    cfg.qn;                     % weight_qn
    cfg.qalpha;                 % weight_qalpha
    cfg.qjerk;                  % weight_qjerk
    cfg.qddelta;                % weight_ddelta
    cfg.alat_max;               % a_lat_max  <<--- usa o do YAML
    cfg.a_min;                  % a_long_min
    cfg.a_max;                  % a_long_max
    cfg.track_safety_margin;    % safety_margin
    cfg.overtake_d;              % overtake_d
];

end
