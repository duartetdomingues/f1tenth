function solver = setup_ocp_mpc_curv_v2(N, Ts, R, track)

    import casadi.*

    [model , constraints] = car_model_mpc_curv_v2(Ts, R, track);

    %% === OCP setup ===
    ocp = AcadosOcp();
    ocp.model = model;

    % Horizonte de predição
    ocp.solver_options.N_horizon = N;
    ocp.solver_options.tf = N * Ts;

    % Cost function (linear least squares)
    %ny = nx + nu;
    %ny_e = nx;
    %ocp.cost.cost_type = 'LINEAR_LS';
    %ocp.cost.cost_type_e = 'LINEAR_LS';
    %ocp.cost.cost_ext_fun_type = 'casadi';
    %ocp.cost.cost_type_0 = 'LINEAR_LS';
    % % Seleção de estados/entradas para o custo in initial cost term
    % ocp.cost.Vx_0 = [eye(nx); zeros(nu, nx)];
    % ocp.cost.Vu_0 = [zeros(nx, nu); eye(nu)];
    % ocp.cost.W_0 = blkdiag(Q, R);
    % 
    %  % Seleção de estados/entradas para o custo in lagrange term
    % ocp.cost.Vx_e = eye(nx);
    % ocp.cost.W_e = Q;
    % 
    % % Seleção de estados/entradas para o custo in terminal cost term
    % ocp.cost.W = blkdiag(Q, R);
    % ocp.cost.Vx = [eye(nx); zeros(nu, nx)];
    % ocp.cost.Vu = [zeros(nx, nu); eye(nu)];

    % === Função de custo não linear ===
    % x_ref = p(1);
    % y_ref = p(2);
    % v_ref = p(3);
    % 
    % e = [x(1)-x_ref;
    %      x(2)-y_ref;
    %      0;
    %      0;
    %      x(5)-v_ref];
    % 
    % j = e' * q * e + u' * r * u;

    x=model.x;
    u=model.u;

    ocp.cost.cost_type_0 = 'NONLINEAR_LS';
    ocp.cost.cost_type = 'NONLINEAR_LS';
    ocp.cost.cost_type_e = 'NONLINEAR_LS';

    model.cost_y_expr_0 = [x(1);x(2);u(1)]; 
    model.cost_y_expr = [x(1);x(2);u(1)]; 
    model.cost_y_expr_e = [x(1);x(2);]; 

    % model.cost_y_expr   = [x(1); x(2); sin(x(3)); cos(x(3)); u(1); u(2)];  % inclui steering
    % model.cost_y_expr_e = [x(1); x(2); sin(x(3)); cos(x(3))];  % inclui steering
    % 
    % Q_pos = diag([Q(1,1), Q(2,2), Q(3,3), Q(3,3), Q(4,4), Q(5,5)]);
    % Q_pos2= diag([Q(1,1), Q(2,2), Q(3,3), Q(3,3)]);
    ocp.cost.W_0 = diag([10, 5, 1]);
    ocp.cost.W   = diag([10, 5, 1]);
    ocp.cost.W_e = diag([10, 5]);

    %ocp.cost.W = 1;      % ou matriz equivalente à usada em `J`
    %ocp.cost.W_e = 1;    % terminal
    %ocp.cost.W_0 = 1;    % inicial

    % ocp.constraints.constr_type = 'BGH';  % <- OBRIGATÓRIO

    % ocp.cost.cost_type_0 = 'EXTERNAL';   % nó 0
    % ocp.cost.cost_type   = 'EXTERNAL';   % nós 1…N-1
    % ocp.cost.cost_type_e = 'EXTERNAL';   % nó N (terminal)


    % Constraints dEntradas
    % ocp.constraints.idxbu = [0;1];
    % ocp.constraints.lbu = [constraints.ddelta_min; constraints.dthrottle_min];
    % ocp.constraints.ubu = [constraints.ddelta_max; constraints.dthrottle_max];
    

    % Constraints Estados
    % ocp.constraints.idxbx = [constraints.delta_idx;constraints.throttle_idx];
    % ocp.constraints.lbx = [constraints.delta_min; constraints.throttle_min];
    % ocp.constraints.ubx = [constraints.delta_max; constraints.throttle_max];



    %servo_max_rate = p(2);       % velocidade angular máxima (rad/s)
    % servo_max_rate = 5.2360/10;
    % 
    % ocp.constraints.lh = -servo_max_rate;
    % ocp.constraints.uh =  servo_max_rate;
    % ocp.constraints.lh_0 = -servo_max_rate;
    % ocp.constraints.uh_0 =  servo_max_rate;
    % 
    % ocp.constraints.lh = constraints.lb_h;
    % ocp.constraints.uh = constraints.ub_h;
    % ocp.constraints.lh_0 = constraints.lb_h;
    % ocp.constraints.uh_0 = constraints.ub_h; 


    ocp.constraints.x0 =ones(5,1)* 1e-1;

    %% === Opções do solver ===
    ocp.solver_options.qp_solver =  'PARTIAL_CONDENSING_HPIPM';
    %ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
    ocp.solver_options.hessian_approx = 'EXACT';


    if (~isempty(ocp.model.f_expl_expr))
        ocp.solver_options.integrator_type = 'ERK';
        disp("integrator_type = ERK")
    elseif (~isempty(ocp.model.disc_dyn_expr))
        ocp.solver_options.integrator_type = 'DISCRETE';
        disp("integrator_type = DISCRETE")
    else 
        disp("integrator_type Error");
        return
    end

    %ocp.solver_options.globalization       = 'MERIT_BACKTRACKING';
    %ocp.solver_options.levenberg_marquardt = 1e-3;
    ocp.solver_options.nlp_solver_max_iter = 200;


    ocp.solver_options.nlp_solver_type = 'SQP'; % ou 'SQP_RTI' para mais rápido
    ocp.solver_options.print_level = 3;

    ocp.solver_options.sim_method_num_stages = 1;
    ocp.solver_options.sim_method_num_steps = 1;


    %% Final validation
    assert(all(size(model.x) == [5,1]), 'x dimension mismatch');
    assert(all(size(model.u) == [2,1]), 'u dimension mismatch');
    assert(all(size(model.f_expl_expr) == [5,1]), 'f_expl dimension mismatch');
    if ~isempty(model.con_h_expr)
        assert(length(constraints.lb_h) == length(model.con_h_expr), 'Constraint dimension mismatch');
    end
    disp('Model validation passed');

    %% === Criar solver ACADOS ===
    opts = struct();
    opts.output_dir = fullfile(pwd, 'c_generated_code');
    opts.save_script = false;
    
    solver = AcadosOcpSolver(ocp, opts);

end
