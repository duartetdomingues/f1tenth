function formulateMPCC(track_length) %TO DO - ADD ELLIPSE CONSTRAINT, look into the interpolation for the track splines, add cost term regarding the slip angle in order to reduce aggressiveness (AMZ), use the derivative of the control input, incorporate lift force(downforce), add cone related constraints, look into the warm start, check for inequalities that depend on consecutive timesteps, might be necessary to take the last ones, tire models need a closer look, add the jacobian (might speed up), experiment with different solvers to see which one performs better
    %% Read Tunable Parameters from File and Store

    % Choose and read config file
    [config_file,config_path] = uigetfile('../config/*.yaml', 'Select a config file');
    global config_data ;
    config_data =  yaml.loadFile([config_path,config_file]);

    % Define Model parameters
    global car
    car.l_f = config_data.model_params.l_f;
    car.l_r = config_data.model_params.l_r;
    car.m = config_data.model_params.m;
    car.I_z = config_data.model_params.I_z;
    car.T_max = config_data.model_params.T_max;
    car.T_brake = config_data.model_params.T_brake;
    car.GR = config_data.model_params.GR;
    car.eta_motor = config_data.model_params.eta_motor;
    car.r_wheel = config_data.model_params.r_wheel;
    car.g = config_data.model_params.g;
    car.C_roll = config_data.model_params.C_roll;
    car.rho = config_data.model_params.rho;
    car.lambda_blend_min = config_data.model_params.lambda_blend_min;
    car.lambda_blend_max = config_data.model_params.lambda_blend_max;
    car.C_d = config_data.model_params.C_d;
    car.C_l = config_data.model_params.C_l;
    car.downforce_front = config_data.model_params.C_d;
    car.downforce_rear = config_data.model_params.C_l;


    % Define Tyre parameters
    car.B = config_data.tyre_params.B;
    car.C = config_data.tyre_params.C;
    car.D = config_data.tyre_params.D;

    %% Define MPC fields
    model = {};

    % State Space Dimensions
    n_input = 2; % throttle(d), steering (delta)
    additional_variables = 4; % s(t), s(t-1) , d(t+1)-d(t), delta(t+1) - delta(t)
    n_state = 6; % X, Y Psi, vx, vy, r

    model.nvar = n_input + additional_variables + n_state; % z variable
    model.neq = n_input + n_state + 1 + 1; % state dynamics + input dynamics + progress dynamics + variation of progress

    % z =  [1 - s(t-1) 
    %       2 - d(t+1) - d(t)
    %       3 - delta(t+1) - delta(t)
    %       4 - s(t) 
    %       5 - d(t)
    %       6 - delta(t)
    %       7 - X 
    %       8 - Y 
    %       9 - Psi 
    %       10 - vx 
    %       11 - vy 
    %       12 - r]
    % ==   [s(t+1)-s(t) u(t+1)-u(t) s(t) u(t) x(t)]

%     n_objective_params = 10; % objective parameters
%     n_inequalities_params = 12; % inequalities
    n_params = config_data.mpc_params.objective.p; % parameters

%     model.npar = n_params; % parameters

    % p =  [1 - N
    %       2 - alpha_CL
    %       3 - alpha_L
    %       4 - q_v_y 
    %       5 - e_CL_exp
    %       6 - d_max
    %       7 - beta_steer
    %       8 - lambda
    %       9 - v_max 
    %       10 - q_v_max]

    N = config_data.mpc_params.objective.N;
    model.N = N;
    global integrator_stepsize
    integrator_stepsize = 1/config_data.controller_freq;

    %% Define Objective Function

    % First timestep
    model.objective{1} = @objectiveInitial;
    % In between timesteps
    for i=2:N-2
        model.objective{i} = @objectiveMiddle;
    end
    % Penultimate
    model.objective{N-1} = @objectivePenultimate;
    % Last
    model.objective{N} = @objectiveLast;
    
    %% Dynamics (equalities)

    model.eq = @(z) [z(4); progress_dynamics(z); z(2)+z(5); z(3)+z(6); vehicle_dynamics(z)]; % State Transition
    model.E = [[1;0;0;0;0;0;0;0;0;0] zeros(10,2) [0 0 0 0 0 0 0 0 0; eye(9)]]; % Matrix that chooses z that will update with State Transition
    model.xinitidx = [1,4,7,8,9,10,11,12];  % States updated in the beginning of a new optimization

    %% Inequalities

    % Defining upper and lower bounds for inequalities
    throttle_min = config_data.mpc_params.ineq_limits.throttle_min;
    throttle_max = config_data.mpc_params.ineq_limits.throttle_max;
    steering_min = config_data.mpc_params.ineq_limits.steering_min;
    steering_max = config_data.mpc_params.ineq_limits.steering_max;
    delta_throttle_min = config_data.mpc_params.ineq_limits.delta_throttle_min;
    delta_throttle_max = config_data.mpc_params.ineq_limits.delta_throttle_max;
    delta_steering_min = config_data.mpc_params.ineq_limits.delta_steering_min;
    delta_steering_max = config_data.mpc_params.ineq_limits.delta_steering_max;
    e_CL_min = config_data.mpc_params.ineq_limits.e_CL_min;
    e_CL_max = config_data.mpc_params.ineq_limits.e_CL_max;
    delta_s_min = config_data.mpc_params.ineq_limits.delta_s_min;
    delta_s_max = config_data.mpc_params.ineq_limits.delta_s_max;  

    % Simple Inequalities
    %          [s(t-1)  u(t+1)-u(t)                             s(t)          u(t)                        x                                 ] 
    model.lb = [-inf,   delta_throttle_min, delta_steering_min, 0,            throttle_min, steering_min, -inf, -inf, -inf, -inf, -inf, -inf];
    model.ub = [+inf,   delta_throttle_max, delta_steering_max, track_length, throttle_max, steering_max, +inf, +inf, +inf, +inf, +inf, +inf];

    % Nonlinear Inequalitie
    model.nh = 2;
    model.ineq = @ineq_e_cl;
    model.hl = [delta_s_min, e_CL_min];
    model.hu = [delta_s_max, e_CL_max];

    %% Generate Solver 

    % Make the output a continuous vector
    outputs = newOutput('z',1:model.N,1:model.nvar);

    codeoptions = getOptions('solverMPCC');
    codeoptions.platform = 'Gnu-x86_64'; % to specify the platform
    % codeoptions.mip.timeout = integrator_stepsize;
    % codeoptions.printlevel = 1;         % printlevel should always be set to 0 when recording performance timings or when deploying the code on an autonomous embedded system.
    % codeoptions.cleanup = 0;            % to keep necessary files for target compile
    codeoptions.maxit = 1000000;            % maximum number of iterations (to control node processing time)
    codeoptions.optlevel = 3;           % compiler optimizationLearn-to-Race level: set to 3 when generating code for deployment or timing measurements.
    % codeoptions.threadSafeStorage = true;
    % codeoptions.timing = 1;             % measure computation time. Timing should be turned off when deploying the code on an autonomous embedded system.
    % codeoptions.embedded_timing = 1;    % by default when choosing to generate solvers for target platforms, timing is disabled.
    % codeoptions.solver_timeout = 0;     % setting the option to 1 will enable the timeout and provide the floating point variable solver_timeout as a runtime parameter.
    codeoptions.BuildSimulinkBlock = 0; % set to 0 to skip the build of simulink s-function
    % codeoptions.nlp.compact_code = 1;   % set to 1 to enable the FORCESPRO server to generate smaller code, which results in shorter compilation time and slightly better solve time in some cases.
    codeoptions.parallel = 1;           % set to 1 to enable multicore computation or to >1 to set maximum number of threads
    codeoptions.nlp.ad_tool = 'casadi-3.5.5'; % 'symbolic-math-tbx'; %'casadi351';
    codeoptions.solver_timeout = 1; % Setting the option to 1 will enable the timeout and provide the floating point variable solver_timeout as a runtime parameter. Setting the option to 2 will additionally provide the floating point variable timeout_estimate_coeff as a runtime parameter.
    codeoptions.printlevel = 2; % on some platforms printing is not supported
    codeoptions.cleanup = 0; % to keep necessary files for targe compiling
    codeoptions.overwrite = 1;
    % codeoptions.warmstart = 1;
    % codeoptions.init = 2;
    codeoptions.sse = 1; % SIMD usage
    % codeoptions.avx = 1; % AVX
    % codeoptions.nlp.integrator = integrator_stepsize;
    % nodes = 4;
    % codeoptions.nlp.integrator.nodes = nodes;
    % codeoptions.nlp.integrator.type = 'ERK4';
    % codeoptions.nlp.integrator.attempt_subsystem_exploitation = 1; % Enable subsystem exploitation for performance   
    % codeoptions.optimize_choleskydivision = 1;
    % codeoptions.optimize_registers = 1;
    % codeoptions.optimize_uselocalsall = 1;
    % codeoptions.optimize_uselocalsheavy = 1; % overriden if uselocalsall is enabled
    % codeoptions.optimize_uselocalssimple = 1; % overriden if uselocalsheavy is enabled
    % codeoptions.optimize_operationsrearrange = 1;
    % codeoptions.optimize_loopunrolling = 1;
    % codeoptions.optimize_enableoffset = 1;
    % codeoptions.nlp.compact_code = 1; % The server failed to generate Code and returned the following message: Problem compiling the solver when using the web compiler. Please contact support@embotech.com.
    % codeoptions.floattype = 'float'; deu erro O.o
    codeoptions.noVariableElimination = 1; 
    
    % Generate solver
    FORCES_NLP(model, codeoptions, outputs);

end

%% AUXILIARY FUNCTIONS

%  Objetive functions
% Initial
function f = objectiveInitial(z)
    global config_data

    alpha_CL = config_data.mpc_params.objective.alpha_CL;
    alpha_L = config_data.mpc_params.objective.alpha_L;
    q_v_y = config_data.mpc_params.objective.q_v_y;
    e_CL_exp = config_data.mpc_params.objective.e_CL_exp;
    d_max = config_data.mpc_params.objective.d_max;
    beta_steer = config_data.mpc_params.objective.beta_steer;
    lambda = config_data.mpc_params.objective.lambda;
    v_max = config_data.mpc_params.objective.v_max;
    q_v_max = config_data.mpc_params.objective.q_v_max;

%     alpha_CL = p(2);
%     alpha_L = p(3);
%     q_v_y = p(4);
%     e_CL_exp = p(5);
%     d_max = p(6);
%     beta_steer = p(7);
%     lambda = p(8);
%     v_max = p(9);
%     q_v_max = p(10);

    [g_x, g_y, dg_x, dg_y] = track(z(4));

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;
    
    e_CL = dg_y*(z(7) - g_x) - dg_x*(z(8) - g_y);
    e_L = -dg_x*(z(7) - g_x) - dg_y*(z(8) - g_y);

    f = alpha_CL*e_CL^e_CL_exp + alpha_L*e_L^2 + beta_steer*z(3)^2;
end
% Middle
function f = objectiveMiddle(z)
    global config_data
%     alpha_CL = p(2);
%     alpha_L = p(3);
%     q_v_y = p(4);
%     e_CL_exp = p(5);
%     d_max = p(6);
%     beta_steer = p(7);
%     lambda = p(8);
%     v_max = p(9);
%     q_v_max = p(10);
    alpha_CL = config_data.mpc_params.objective.alpha_CL;
    alpha_L = config_data.mpc_params.objective.alpha_L;
    q_v_y = config_data.mpc_params.objective.q_v_y;
    e_CL_exp = config_data.mpc_params.objective.e_CL_exp;
    d_max = config_data.mpc_params.objective.d_max;
    beta_steer = config_data.mpc_params.objective.beta_steer;
    lambda = config_data.mpc_params.objective.lambda;
    v_max = config_data.mpc_params.objective.v_max;
    q_v_max = config_data.mpc_params.objective.q_v_max;

    [g_x, g_y, dg_x, dg_y] = track(z(4));

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;
    
    e_CL = dg_y*(z(7) - g_x) - dg_x*(z(8) - g_y);
    e_L = -dg_x*(z(7) - g_x) - dg_y*(z(8) - g_y);

    f = q_v_y*z(11)^2 + alpha_CL*e_CL^e_CL_exp...
        + alpha_L*e_L^2 + beta_steer*(z(3))^2 + exp(q_v_max*(z(10)-v_max));
end
% Penultimate
function f = objectivePenultimate(z)
    global config_data
%     alpha_CL = p(2);
%     alpha_L = p(3);
%     q_v_y = p(4);
%     e_CL_exp = p(5);
%     d_max = p(6);
%     beta_steer = p(7);
%     lambda = p(8);
%     v_max = p(9);
%     q_v_max = p(10);
    alpha_CL = config_data.mpc_params.objective.alpha_CL;
    alpha_L = config_data.mpc_params.objective.alpha_L;
    q_v_y = config_data.mpc_params.objective.q_v_y;
    e_CL_exp = config_data.mpc_params.objective.e_CL_exp;
    d_max = config_data.mpc_params.objective.d_max;
    beta_steer = config_data.mpc_params.objective.beta_steer;
    lambda = config_data.mpc_params.objective.lambda;
    v_max = config_data.mpc_params.objective.v_max;
    q_v_max = config_data.mpc_params.objective.q_v_max;

    [g_x, g_y, dg_x, dg_y] = track(z(4));

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;
    
    e_CL = dg_y*(z(7) - g_x) - dg_x*(z(8) - g_y);
    e_L = -dg_x*(z(7) - g_x) - dg_y*(z(8) - g_y);

    f = q_v_y*z(11)^2 + alpha_CL*e_CL^e_CL_exp...
        + alpha_L*e_L^2 + exp(q_v_max*(z(10)-v_max));
end
% Last
function f = objectiveLast(z)
    global config_data
%     alpha_CL = p(2);
%     alpha_L = p(3);
%     q_v_y = p(4);
%     e_CL_exp = p(5);
%     d_max = p(6);
%     beta_steer = p(7);
%     lambda = p(8);
%     v_max = p(9);
%     q_v_max = p(10);
    alpha_CL = config_data.mpc_params.objective.alpha_CL;
    alpha_L = config_data.mpc_params.objective.alpha_L;
    q_v_y = config_data.mpc_params.objective.q_v_y;
    e_CL_exp = config_data.mpc_params.objective.e_CL_exp;
    d_max = config_data.mpc_params.objective.d_max;
    beta_steer = config_data.mpc_params.objective.beta_steer;
    lambda = config_data.mpc_params.objective.lambda;
    v_max = config_data.mpc_params.objective.v_max;
    q_v_max = config_data.mpc_params.objective.q_v_max;

    [g_x, g_y, dg_x, dg_y] = track(z(4));

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;
    
    e_CL = dg_y*(z(7) - g_x) - dg_x*(z(8) - g_y);
    e_L = -dg_x*(z(7) - g_x) - dg_y*(z(8) - g_y);

    f = -lambda*z(4) + q_v_y*z(11)^2 + alpha_CL*e_CL^e_CL_exp...
            + alpha_L*e_L^2 + exp(q_v_max*(z(10)-v_max));
end

function progress = progress_dynamics(z)
    global integrator_stepsize
    progress_old = z(4);
    yaw = z(9);
    vx = z(10);
    vy = z(11);

    [~, ~, dg_x, dg_y] = track(progress_old);
    dg = [dg_x; dg_y];
    v = [vx; vy];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];

    vk = dg'*R*v;
    progress = progress_old + vk/integrator_stepsize;
end


%% 
% Blended Bicicle Model
function blended = vehicle_dynamics(z)
    global car
    global integrator_stepsize

    % discritized vehicle model
    dyn_bike = RK4(z(7:12),z(5:6),@dynamic_bicycle_model, integrator_stepsize);
    kin_bike = RK4(z(7:12),z(5:6),@kinematic_bicycle_model, integrator_stepsize);
    
    % blend the two models
    lambda_ = min(max((norm(z([10,11])) - car.lambda_blend_max)/(car.lambda_blend_max - car.lambda_blend_min), 0), 1);
    
%     blended = lambda_*dyn_bike + (1-lambda_)*kin_bike;
    blended = dyn_bike;

end

% Dynamic Bicycle Model
function dX = dynamic_bicycle_model(x,u)
    global car

    % x = [1 - X
    %      2 - Y
    %      3 - Psi
    %      4 - vx
    %      5 - vy
    %      6 - r]

    % u = [1 - d 
    %      2 - delta]

    alpha_f = atan2(x(5) + x(6)*car.l_f, x(4)) - u(2);
    alpha_r = atan2(x(5) - x(6)*car.l_r, x(4));
    Ff_y = -2*car.D*sin(car.C*atan(car.B*alpha_f));
    Fr_y = -2*car.D*sin(car.C*atan(car.B*alpha_r));
    
    t_bool = (u(1) < 0);
    T = car.T_max + t_bool*(car.T_brake - car.T_max);
    F_prop = 2*car.eta_motor*T*car.GR/car.r_wheel*u(1);
    
    F_roll = car.C_roll*car.m*car.g;
    
    F_drag = 1/2*car.rho*car.C_d*x(4)^2;
    
    cos_delta = cos(u(2));
    F_x = F_prop - F_roll - F_drag - Ff_y*sin(u(2));
    F_y = Ff_y*cos_delta + Fr_y;
    M_z = Ff_y*cos_delta*car.l_f - Fr_y*car.l_r;
    
    % State derivative function
    cos_Phi = cos(x(3));
    sin_Phi = sin(x(3));
    dX = [x(4)*cos_Phi - x(5)*sin_Phi;
          x(4)*sin_Phi + x(5)*cos_Phi;
          x(6);
          1/car.m*( F_x ) + x(5)*x(6);
          1/car.m*( F_y ) - x(4)*x(6);
          1/car.I_z*( M_z )];
end

% Kinematic Bicycle Model
function dX = kinematic_bicycle_model(x,u)
    global car

    % x = [1 - X
    %      2 - Y
    %      3 - Psi
    %      4 - vx
    %      5 - vy
    %      6 - r]

    % u = [1 - d 
    %      2 - delta]

    t_bool = (u(1) < 0);
    T = car.T_max + t_bool*(car.T_brake - car.T_max);
    F_prop = 2*car.eta_motor*T*car.GR/car.r_wheel*u(1);
    
    F_roll = car.C_roll*car.m*car.g;
    
    F_drag = 1/2*car.rho*car.C_d*x(4)^2;

    F_x = F_prop - F_roll - F_drag;
    
    % State derivative function
    cos_Phi = cos(x(3));
    sin_Phi = sin(x(3));

    dX = [x(4)*cos_Phi - x(5)*sin_Phi;
          x(4)*sin_Phi + x(5)*cos_Phi;
          x(4)/(car.l_r + car.l_f)*tan(u(2));
          F_x/car.m;
          F_x/car.m*tan(u(2))*car.l_r/(car.l_r+car.l_f);
          F_x/car.m/(car.l_r+car.l_f)*tan(u(2))];
end

% Nonlinear Inequalities
function h = ineq_e_cl(z)
    [g_x, g_y, dg_x, dg_y] = track(z(4));

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;
    
    h = [dg_y*(z(7) - g_x) - dg_x*(z(8) - g_y); % e_CL
         z(4)-z(1)]; % s(t) - s(t-1)
end

%% ----------- Dump code for future references --------- IGNORE ---------
% Define MPC parameters
%     % Objective
%     N = config_data.mpc_params.objective.N;
%     alpha_CL = config_data.mpc_params.objective.alpha_CL;
%     alpha_L = config_data.mpc_params.objective.alpha_L;
%     q_v_y = config_data.mpc_params.objective.q_v_y;
%     e_CL_exp = config_data.mpc_params.objective.e_CL_exp;
%     beta_steer = config_data.mpc_params.objective.beta_steer;
%     lambda = config_data.mpc_params.objective.lambda;
%     v_max = config_data.mpc_params.objective.v_max ;
%     q_v_max = config_data.mpc_params.objective.q_v_max;
%     % Inequalities
%     u_min = config_data.mpc_params.ineq_limits.u_min;
%     u_max = config_data.mpc_params.ineq_limits.u_max;
%     delta_u_min = config_data.mpc_params.ineq_limits.delta_u_min;
%     delta_u_max = config_data.mpc_params.ineq_limits.delta_u_max;
%     e_CL_min = config_data.mpc_params.ineq_limits.e_CL_min;
%     e_CL_max = config_data.mpc_params.ineq_limits.e_CL_max;
%     delta_s_min = config_data.mpc_params.ineq_limits.delta_s_min;
%     delta_s_max = config_data.mpc_params.ineq_limits.delta_s_max;

% % First timestep
%     model.objective{1} = @(z,p) p(1)*e_CL^p(5) + p(2)*e_L^2;
%     %In between timesteps
%     for i=2:model.N-1
%         model.objective{i} = @(z,p) p(4)*z(11)^2 + p(1)*e_CL^p(5)...
%             + p(2)*e_L^2 + p(7)*(z(2)-z(4)) + exp(p(10)*(z(10)-p(9)));
%     end
%     %Last timestep
%     model.objective{N} = @(z,p) -p(8)*z(7) + p(4)*z(11)^2 + p(1)*e_CL^p(5)...
%             + p(2)*e_L^2 + exp(p(10)*(z(10)-p(9)));

%     alpha_CL = p(2);
%     alpha_L = p(3);
%     q_v_y = p(4);
%     e_CL_exp = p(5);
%     d_max = p(6);
%     beta_steer = p(7);
%     lambda = p(8);
%     v_max = p(9);
%     q_v_max = p(10);

% First timestep
%     model.objective{1} = @(z,p) alpha_CL*e_CL^e_CL_exp + alpha_L*e_L^2;
%     %In between timesteps
%     for i=2:N-1
%         model.objective{i} = @(z,p) q_v_y*z(11)^2 + alpha_CL*e_CL^e_CL_exp...
%             + alpha_L*e_L^2 + beta_steer*(z(4))^2 + exp(q_v_max*(z(10)-v_max));
%     end
%     %Last timestep
%     model.objective{N} = @(z,p) -lambda*z(1) + q_v_y*z(11)^2 + alpha_CL*e_CL^e_CL_exp...
%             + alpha_L*e_L^2 + exp(q_v_max*(z(10)-v_max));

%     [g_x, g_y, dg_x, dg_y] = track(z(4));

% 
%     temp = sqrt(dg_x^2 + dg_y^2);
%     dg_x = dg_x/temp;
%     dg_y = dg_y/temp;
%     
%     e_CL = dg_y*(z(7) - g_x) - dg_x*(z(8) - g_y);
%     e_L = -dg_x*(z(7) - g_x) - dg_y*(z(8) - g_y);
% 
%     % First timestep
%     model.objective{1} = @(z,p) alpha_CL*e_CL^e_CL_exp + alpha_L*e_L^2;
%     %In between timesteps
%     for i=2:N-1
%         model.objective{i} = @(z,p) q_v_y*z(11)^2 + alpha_CL*e_CL^e_CL_exp...
%             + alpha_L*e_L^2 + beta_steer*(z(3))^2 + exp(q_v_max*(z(10)-v_max));
%     end
%     %Last timestep
%     model.objective{N} = @(z,p) -lambda*z(1) + q_v_y*z(11)^2 + alpha_CL*e_CL^e_CL_exp...
%             + alpha_L*e_L^2 + exp(q_v_max*(z(10)-v_max));
%     throttle_min = p(11);
%     throttle_max = p(12);
%     steering_min = p(13);
%     steering_max = p(14);
%     delta_throttle_min = p(15);
%     delta_throttle_max = p(16);
%     delta_steering_min = p(17);
%     delta_steering_max = p(18);
%     e_CL_min = p(19);
%     e_CL_max = p(20);
%     delta_s_min = p(21);
%     delta_s_max = p(22);

    % p =  [1 - N
    %       2 - alpha_CL
    %       3 - alpha_L
    %       4 - q_v_y 
    %       5 - e_CL_exp
    %       6 - d_max
    %       7 - beta_steer
    %       8 - lambda
    %       9 - v_max 
    %       10 - q_v_max 
    %       11 - throttle_min 
    %       12 - throttle_max
    %       13 - steering_min
    %       14 - steering_max
    %       15 - delta_throttle_min
    %       16 - delta_throttle_max
    %       17 - delta_steering_min
    %       18 - delta_steering_max 
    %       19 - e_CL_min
    %       20 - e_CL_max
    %       21 - delta_s_min
    %       22 - delta_s_max]