function ocp = setup_ocp(N, Ts, Q, R, delta_max, v_max)

    import casadi.*
    model = ocp_model_acados(); % <- Verifica se tens este ficheiro

    % Create OCP object
    ocp_model = acados_ocp_model();
    ocp_model.set('name', 'mpc_model');
    ocp_model.set('T', (N * Ts));  % Prediction horizon
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dim_nx', 5);
    ocp_model.set('dim_nu', 2);
    ocp_model.set('dim_np', 0);
    ocp_model.set('dim_ny', 7);     % states + inputs
    ocp_model.set('dim_ny_e', 5);   % terminal cost (states only)
    ocp_model.set('dyn_expr_f', model.f_expl_expr);

    % Cost
    W = blkdiag(Q, R);       % stage cost: states + inputs
    W_e = Q;                 % terminal cost: only states
    ocp_model.set('cost_type', 'linear_ls');
    ocp_model.set('cost_type_e', 'linear_ls');
    ocp_model.set('cost_W', W);
    ocp_model.set('cost_W_e', W_e);

    % Output selection
    Vx = eye(5);     % Select all states
    Vu = eye(2);     % Select all controls
    ocp_model.set('cost_Vx', [Vx, zeros(5,2)]);
    ocp_model.set('cost_Vu', [zeros(2,5), Vu]);
    ocp_model.set('cost_Vx_e', Vx);

    % Constraints
    ocp_model.set('constr_lbu', [-delta_max; -v_max]);
    ocp_model.set('constr_ubu', [delta_max; v_max]);
    ocp_model.set('constr_C', eye(2));
    ocp_model.set('constr_D', zeros(2,5));
    ocp_model.set('constr_lg', []);
    ocp_model.set('constr_ug', []);

    ocp_model.set('dim_nbu', 2);  % box constraints on inputs

    ocp = acados_ocp();
    ocp.set_model(ocp_model);

    opts = acados_ocp_opts();
    opts.set('compile_interface', 'auto');
    opts.set('codgen_model', true);
    opts.set('param_scheme_N', N);
    opts.set('nlp_solver', 'sqp');         % could be 'sqp_rti' too
    opts.set('sim_method', 'erk');         % explicit RK
    opts.set('qp_solver', 'full_condensing_hpipm');

    ocp.set_opts(opts);
end
