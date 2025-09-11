function [solver, model_var] = setup_ocp_mpc_curv(N,p ,Ts, track)

    import casadi.*

    [model , constraints, model_var] = car_model_mpc_curv(Ts, track);

    %% === OCP setup ===
    ocp = AcadosOcp();
    ocp.model = model;

    % Horizonte de predição
    ocp.solver_options.N_horizon = N;
    ocp.solver_options.tf = N * Ts;

    ocp.cost.cost_type_0 = 'EXTERNAL';   % nó 0
    ocp.cost.cost_type   = 'EXTERNAL';   % nós 1…N-1
    ocp.cost.cost_type_e = 'EXTERNAL';   % nó N (terminal)


    % Constraints Entradas
    % ocp.constraints.idxbu = [0;1];
    % ocp.constraints.lbu = [constraints.ddelta_min; constraints.dthrottle_min];
    % ocp.constraints.ubu = [constraints.ddelta_max; constraints.dthrottle_max];
    

    % Constraints Estados
    idxbx = [constraints.vx_idx;constraints.heading_idx;constraints.delta_idx;constraints.throttle_idx];
    lbx = [ constraints.vx_min;constraints.heading_min;constraints.delta_min; constraints.throttle_min];
    ubx = [constraints.vx_max;constraints.heading_max;constraints.delta_max; constraints.throttle_max];

    ocp.constraints.idxbx = idxbx;
    ocp.constraints.lbx = lbx;
    ocp.constraints.ubx = ubx;


    ocp.constraints.idxbx_e = idxbx;
    ocp.constraints.lbx_e = lbx;
    ocp.constraints.ubx_e = ubx;

    ocp.constraints.idxbx_0 = idxbx;
    ocp.constraints.lbx_0 = lbx;
    ocp.constraints.ubx_0 = ubx;

    % ocp.constraints.idxbx = [constraints.heading_idx];
    % ocp.constraints.lbx = [constraints.heading_min];
    % ocp.constraints.ubx = [constraints.heading_max];
    % 
    % ocp.constraints.idxbx_e = [constraints.heading_idx];
    % ocp.constraints.lbx_e = [constraints.heading_min];
    % ocp.constraints.ubx_e = [constraints.heading_max];
    % 
    % ocp.constraints.idxbx_0 = [constraints.heading_idx];
    % ocp.constraints.lbx_0 = [constraints.heading_min];
    % ocp.constraints.ubx_0 = [constraints.heading_max];


    if (~isempty(ocp.model.con_h_expr))
        ocp.constraints.lh = constraints.lb_h;
        ocp.constraints.uh = constraints.ub_h;  
    end

    if (~isempty(ocp.model.con_h_expr_0))
        ocp.constraints.lh_0 = constraints.lb_h;
        ocp.constraints.uh_0 = constraints.ub_h; 
    end


    %servo_max_rate = p(2);       % velocidade angular máxima (rad/s)
    % servo_max_rate = 5.2360/10;
    % 
    % ocp.constraints.lh = -servo_max_rate;
    % ocp.constraints.uh =  servo_max_rate;
    % ocp.constraints.lh_0 = -servo_max_rate;
    % ocp.constraints.uh_0 =  servo_max_rate;
 
   %ocp.constraints.x0=zeros(8,1);
   %ocp.constraints.x0 = ones(8,1)* 1e-1;
   % Estados       x = [s; n; µ; vx; vy; r; δ; T]
   ocp.constraints.x0 =[0.1;0.1;0.1;0.1;0.1;0.0;0.0;0.0 ];
   % 
   %   ocp.constraints.x0 = [20.5567
   % -0.0060
   %  0.0441
   %  3.2940
   % -0.2095
   %  1.3338
   % -0.0492
   %  0.1000];

   ocp.parameter_values=p;

    %% === Opções do solver ===
   
    %ocp.solver_options.integrator_type = "DISCRETE";

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

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
    %ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM';
    %ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES';
    %ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM';


    %ocp.solver_options.hessian_approx ='GAUSS_NEWTON';
    ocp.solver_options.hessian_approx = 'EXACT';
    ocp.solver_options.nlp_solver_type = 'SQP'; % ou 'SQP_RTI' para mais rápido
    %ocp.solver_options.nlp_solver_type = 'SQP_RTI'; % ou 'SQP_RTI' para mais rápido

    %ocp.solver_options.regularize_method = 'NO_REGULARIZE';
    %ocp.solver_options.regularize_method = 'NO_REGULARIZE';
    %ocp.solver_options.regularize_method = 'GERSHGORIN_LEVENBERG_MARQUARDT';
    ocp.solver_options.regularize_method = 'MIRROR';
    %ocp.solver_options.regularize_method = 'CONVEXIFY';


    % ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM';
    % ocp.solver_options.hessian_approx = 'EXACT';
    % ocp.solver_options.nlp_solver_type = 'SQP_RTI'; % ou 'SQP_RTI' para mais rápido
    
   
    % ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
    % ocp.solver_options.nlp_solver_type = 'SQP_RTI'; % ou 'SQP_RTI' para mais rápido
    % ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
    
    ocp.solver_options.sim_method_num_stages = 4;
    ocp.solver_options.sim_method_num_steps = 4;

    ocp.solver_options.globalization       = 'MERIT_BACKTRACKING';
    %ocp.solver_options.levenberg_marquardt = 1e-3;
    ocp.solver_options.nlp_solver_max_iter = 200;

    %ocp.solver_options.levenberg_marquardt = 1e-3;

    % Ajuste tolerâncias
    %ocp.solver_options.tol = 1e-2;
    % ocp.solver_options.qp_solver_tol_stat = 1e-2;
    % ocp.solver_options.qp_solver_tol_eq=1e-2;
    % ocp.solver_options.qp_solver_tol_ineq=1e-2;
    % ocp.solver_options.qp_solver_tol_comp=1e-2;
    
    ocp.solver_options.print_level = 3;
 



    %% Final validation
    assert(all(size(model.x) == [model_var.nx,1]), 'x dimension mismatch');
    assert(all(size(model.u) == [model_var.nu,1]), 'u dimension mismatch');
    assert(all(size(model.f_expl_expr) == [model_var.nx,1]), 'f_expl dimension mismatch');
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
