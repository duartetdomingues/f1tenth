function solver = setup_ocp(N, Ts, Q, R, delta_max, v_max)

    import casadi.*

    %% === Modelo dinâmico ===
    model = AcadosModel();
    model.name = 'mpc_model';

    nx = 5;  % Estados: [x, y, psi, theta, v]
    nu = 2;  % Controlo: [delta, v]

    x = SX.sym('x', nx);
    u = SX.sym('u', nu);

    L = 0.35;

    f_expl = [ ...
        x(5)*cos(x(3));        % dx/dt
        x(5)*sin(x(3));        % dy/dt
        (x(5)/L)*tan(u(1));    % dpsi/dt
        0;
        0];                    % dtheta e dv são nulos (v é entrada direta)

    model.x = x;
    model.u = u;
    model.f_expl_expr = f_expl;

    %% === OCP setup ===
    ocp = AcadosOcp();
    ocp.model = model;

    % Horizonte de predição
    ocp.solver_options.N_horizon = N;
    ocp.solver_options.tf = N * Ts;

    % Cost function (linear least squares)
    ny = nx + nu;
    ny_e = nx;

    ocp.cost.cost_type = 'LINEAR_LS';
    ocp.cost.cost_type_e = 'LINEAR_LS';
    %ocp.cost.cost_ext_fun_type = 'casadi';
    ocp.cost.cost_type_0 = 'LINEAR_LS';


    ocp.cost.W_0 = blkdiag(Q, R);
    ocp.cost.W_e = Q;

    % Seleção de estados/entradas para o custo
    ocp.cost.Vx_0 = [eye(nx); zeros(nu, nx)];
    ocp.cost.Vu_0 = [zeros(nx, nu); eye(nu)];
    ocp.cost.Vx_e = eye(nx);

    ocp.cost.W = blkdiag(Q, R);
    ocp.cost.Vx = [eye(nx); zeros(nu, nx)];
    ocp.cost.Vu = [zeros(nx, nu); eye(nu)];

    % Constraints (caixas nas entradas)
    ocp.constraints.lbu = [-delta_max; -v_max];
    ocp.constraints.ubu = [ delta_max;  v_max];
    ocp.constraints.idxbu = 0:nu-1;

    % Estado inicial (a definir no runtime)
    ocp.constraints.x0 = zeros(nx,1);

    %% === Opções do solver ===
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM';
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
    ocp.solver_options.integrator_type = 'ERK';
    ocp.solver_options.nlp_solver_type = 'SQP'; % ou 'SQP_RTI' para mais rápido
    ocp.solver_options.print_level = 0;

    ocp.solver_options.sim_method_num_stages = 4;
    ocp.solver_options.sim_method_num_steps = 3;

    %% === Criar solver ACADOS ===
    opts = struct();
    opts.output_dir = fullfile(pwd, 'c_generated_code');
    opts.save_script = false;
    
    solver = AcadosOcpSolver(ocp, opts);

end
