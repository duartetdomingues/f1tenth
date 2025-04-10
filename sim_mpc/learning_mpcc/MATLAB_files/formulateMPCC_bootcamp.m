function formulateMPCC_bootcamp(store_mex_data,differentailCoeffs) %TO DO - ADD ELLIPSE CONSTRAINT, add cost term regarding the slip angle in order to reduce aggressiveness (AMZ), use the derivative of the control input, incorporate lift force(downforce), add cone related constraints, look into the warm start, check for inequalities that depend on consecutive timesteps, might be necessary to take the last ones, tire models need a closer look, add the jacobian (might speed up), experiment with different solvers to see which one performs better, decouple the throttle from the problem and send that command to a PID controller like pure pursuit did

    global config_data
    global diff_Coeffs;
    diff_Coeffs = differentailCoeffs;

    %% Read Tunable Parameters from File and Store

    % Defining upper and lower bounds for inequalities
    throttle_min = config_data.mpc_params.ineq_limits.throttle_min;
    throttle_max = config_data.mpc_params.ineq_limits.throttle_max;
    steering_min = config_data.mpc_params.ineq_limits.steering_min;
    steering_max = config_data.mpc_params.ineq_limits.steering_max;

    %% FORMULATION

    model = {};

    N = config_data.mpc_params.objective.N;
    model.N = N;
    global integrator_stepsize
    integrator_stepsize = 1/config_data.controller_freq;

    derivatives = 2; %d_throttle, d_delta
    n_state = 6; % X, Y, Psi, vx, vy, r
    n_input = 2; % throttle, steering
    prev_variables = 3 + 2; % vx_prev, vy_prev, r_prev, throttle_prev, steering_prev
    additional_variables = 1; % s(t)
    slack_variables = 1; % epsilon

    model.npar = config_data.mpc_params.objective.p + config_data.NN_config.NN_params + (config_data.mpc_params.objective.spline-1)*4*2;
    
    % p =  [1 - alpha_CL
    %       2 - alpha_L
    %       3 - q_r 
    %       4 - e_CL_exp
    %       5 - d_max
    %       6 - beta_steer
    %       7 - lambda
    %       8 - v_max 
    %       9 - q_v_max
    %       10 - l_f
    %       11 - l_r    
    %       12 - m
    %       13 - I_z
    %       14 - T_max_front
    %       15 - T_max_rear
    %       16 - T_brake_front
    %       17 - T_brake_rear 
    %       18 - GR
    %       19 - eta_motor
    %       20 - r_wheel
    %       21 - g
    %       22 - C_roll
    %       23 - rho
    %       24 - lambda_blend_min
    %       25 - lambda_blend_max
    %       26 - C_d
    %       27 - C_l
    %       28 - B
    %       29 - C
    %       30 - D
    %       31 - downforce_front
    %       32 - downforce_rear
    %       33 - beta_throttle
    %       34 - h_cog
    %       35 - mu_x
    %       36 - mu_y
    %       37 - e_CL_max
    %       38 - delta_s_max
    %       39 - throttle_max
    %       40 - steering_max
    %       41 - delta_throttle_max
    %       42 - delta_steering_max
    %       43 - width
    %       44 - length
    %       45 - safety_margin 
    %       46 - beta_epsilon
    %       47 - N
    %       48 - controller_freq
    %       49 - NN_Flag
    %       50 - p (number of parameters)
    %       51 - beta_psi
    %       52 - track_width
    %       53 - diff_gain
    %       54 - path_length
    %       55:55+NNparams-1 - NN_weights (1991 parameters in model 18; 255 in model 22; 78 in model 23)
    %       55+NNparams:55+NNparams+(splinepoints-1)*4*2 - spline coeffs

    %% First timestep

    % z =  [1 - d_d(t)
    %       2 - d_delta(t)
    %       3 - epsilon(t)
    %       4 - d(t)
    %       5 - delta(t)
    %       6 - d(t-1)
    %       7 - delta(t-1)
    %       8 - vx(t-1)
    %       9 - vy(t-1)
    %       10 - r(t-1)
    %       11 - s(t)
    %       12 - X 
    %       13 - Y 
    %       14 - Psi 
    %       15 - vx 
    %       16 - vy 
    %       17 - r]

    % Problem dimensions
    model.nvar(1) = n_input + additional_variables + derivatives + n_state + prev_variables + slack_variables; % input + all additional states minus last progress + states
    model.xinitidx = 4:17;
    model.neq(1) =  additional_variables + n_state + n_input + prev_variables;
    model.nh(1) = 3; %input rate constraints + friction ellipse

    % Objective function
    model.objective{1} = @objectiveInitial;

    % State Transition
    model.eq{1} = @(z,p) [euler(z(4),z(1)); euler(z(5),z(2)); z(4); z(5); z(15); z(16); z(17); progress_dynamics(z(11),z(14),z(15),z(16),p); vehicle_dynamics(z(12:17),z(4:7),[p; z(8:10)])];
    model.E{1} = [zeros(model.neq(1),derivatives+slack_variables) eye(model.neq(1))];

    % Simple Inequalities
    %             [d_d(t), d_delta(t), epsilon,  d(t)          delta(t)      d(t-1)   delta(t-1)  vx(t-1) vy(t-1) r(t-1) s(t)    X     Y     Psi   vx    vy    r   ]
    model.lb{1} = [-inf,   -inf,       -inf,     throttle_min, steering_min, -inf,    -inf,       0   ,   -inf,   -inf,  0,      -inf, -inf, -inf, 0   , -inf, -inf];
    model.ub{1} = [+inf,   +inf,       +inf,     throttle_max, steering_max, +inf,    +inf,       +inf,   +inf,   +inf,  +inf ,  +inf, +inf, +inf, +inf, +inf, +inf];

    % Inequalities
    model.ineq{1} = @ineqI;
    model.hl{1} = [-1, -1 , -inf];
    model.hu{1} = [1, 1, 0];

    %% In between timesteps

    % z =  [1 - d_d(t)
    %       2 - d_delta(t)
    %       3 - epsilon(t)
    %       4 - d(t)
    %       5 - delta(t)
    %       6 - d(t-1)
    %       7 - delta(t-1)
    %       8 - vx(t-1)
    %       9 - vy(t-1)
    %       10 - r(t-1)
    %       11 - s(t)
    %       12 - X 
    %       13 - Y 
    %       14 - Psi 
    %       15 - vx 
    %       16 - vy 
    %       17 - r]

    for i=2:N-2
        % Problem dimensions
        model.nvar(i) = n_input + additional_variables + derivatives + n_state + prev_variables + slack_variables; % input + all additional states minus last progress + states
        model.neq(i) =  additional_variables + n_state + n_input + prev_variables;
        model.nh(i) = 3; %input rate constraints + friction

        % Objective function
        model.objective{i} = @objectiveMiddle;
    
        % State Transition
        model.eq{i} = @(z,p) [euler(z(4),z(1)); euler(z(5),z(2)); z(4); z(5); z(15); z(16); z(17); progress_dynamics(z(11),z(14),z(15),z(16),p); vehicle_dynamics(z(12:17),z(4:7),[p; z(8:10)])];
        if i==N-2
            model.E{i} = [zeros(model.neq(i),slack_variables) eye(model.neq(i))];
        else
            model.E{i} = [zeros(model.neq(i),derivatives+slack_variables) eye(model.neq(i))];
        end
        % Simple Inequalities
        %             [d_d(t), d_delta(t), epsilon,  d(t)          delta(t)      d(t-1)   delta(t-1)  vx(t-1) vy(t-1) r(t-1) s(t)    X     Y     Psi   vx    vy    r   ]
        model.lb{i} = [-inf,   -inf,       -inf,     throttle_min, steering_min, -inf,    -inf,       0   ,   -inf,   -inf,  0,      -inf, -inf, -inf, 0   , -inf, -inf];
        model.ub{i} = [+inf,   +inf,       +inf,     throttle_max, steering_max, +inf,    +inf,       +inf,   +inf,   +inf,  +inf ,  +inf, +inf, +inf, +inf, +inf, +inf];
   
        % Inequalities
        model.ineq{i} = @ineqM;
        model.hl{i} = [-1, -1, -inf];
        model.hu{i} = [1, 1, 0];
    end
    
    %% Penultimate (needed for state transition but basically the same as middle)

    % z =   1 - epsilon
    %       2 - d(t)
    %       3 - delta(t)
    %       4 - d(t-1)
    %       5 - delta(t-1)
    %       6 - vx(t-1)
    %       7 - vy(t-1)
    %       8 - r(t-1)
    %       9 - s(t)
    %       10 - X 
    %       11 - Y 
    %       12 - Psi 
    %       13 - vx 
    %       14 - vy 
    %       15 - r]

    % Problem dimensions
    model.nvar(N-1) = n_input + additional_variables + prev_variables + n_state + slack_variables;
    model.neq(N-1) =  additional_variables + n_state + prev_variables-2; %progress, progress(t-1), states
    model.nh(N-1) = 1; % friction ellipse

    % Objective function
    model.objective{N-1} = @objectiveN_1;
   
    % State Transition
    model.eq{N-1} = @(z,p) [z(13); z(14); z(15); progress_dynamics(z(9),z(12),z(13),z(14),p); vehicle_dynamics(z(10:15),z(2:5),[p; z(6:8)])];
    model.E{N-1} = [zeros(model.neq(N-1),slack_variables) eye(model.neq(N-1))];
   
    % Simple Inequalities
    %               [epsilon d(t)          delta(t)      d(t-1)   delta(t-1)  vx(t-1) vy(t-1) r(t-1) s(t)    X     Y     Psi   vx    vy    r   ]
    model.lb{N-1} = [-inf,   throttle_min, steering_min, -inf,    -inf,       0   ,   -inf,   -inf,  0,      -inf, -inf, -inf, 0   , -inf, -inf];
    model.ub{N-1} = [+inf,   throttle_max, steering_max, +inf,    +inf,       +inf,   +inf,   +inf,  +inf ,  +inf, +inf, +inf, +inf, +inf, +inf];
   
    % Inequalities
    model.ineq{N-1} = @ineqN_1;
    model.hl{N-1} = [-inf];
    model.hu{N-1} = [0];

    %% Last

    % z =   1 - epsilon
    %       2 - vx(t-1)
    %       3 - vy(t-1)
    %       4 - r(t-1)
    %       5 - s(t)
    %       6 - X 
    %       7 - Y 
    %       8 - Psi 
    %       9 - vx 
    %       10 - vy 
    %       11 - r]

    % Problem dimensions
    model.nvar(N) = n_state + additional_variables + slack_variables + prev_variables-2;
    model.nh(N) = 1; %friction ellipse

    % Objective function
    model.objective{N} = @objectiveLast;

    % Simple Inequalities
    %             [epsilon, vx(t-1) vy(t-1) r(t-1) s(t)  X    Y     Psi    vx    vy    r   ]
    model.lb{N} = [-inf,    0   ,   -inf,   -inf,  -inf, -inf, -inf, -inf, 0   , -inf, -inf];
    model.ub{N} = [+inf,    +inf,   +inf,   +inf,  +inf, +inf, +inf, +inf, +inf, +inf, +inf];

    % Inequalities
    model.ineq{N} = @ineqN;
    model.hl{N} = [-inf];
    model.hu{N} = [0];


    %% Generate Solver 

    % Make the output a continuous vector
    outputs(1) = newOutput('z',1:model.N-2,1:model.nvar(1));
    outputs(2) = newOutput('zN_1',model.N-1,1:model.nvar(N-1));
    outputs(3) = newOutput('zN',model.N,1:model.nvar(N));

    codeoptions = getOptions('solverMPCC');
    codeoptions.printlevel = 0;         % printlevel should always be set to 0 when recording performance timings or when deploying the code on an autonomous embedded system.
    codeoptions.maxit = 1000000;            % maximum number of iterations (to control node processing time)
    codeoptions.optlevel = 3;           % compiler optimizationLearn-to-Race level: set to 3 when generating code for deployment or timing measurements.
    codeoptions.BuildSimulinkBlock = 0; % set to 0 to skip the build of simulink s-function
%     codeoptions.nlp.BarrStrat = 'monotone';
    codeoptions.nlp.linear_solver = 'symm_indefinite';
%     codeoptions.nlp.ad_expression_class = 'MX';
%     codeoptions.nlp.linear_solver = 'normal_eqs';
%     codeoptions.nlp.compact_code = 1;   % set to 1 to enable the FORCESPRO server to generate smaller code, which results in shorter compilation time and slightly better solve time in some cases.
    codeoptions.parallel = 1;           % set to 1 to enable multicore computation or to >1 to set maximum number of threads
    codeoptions.nlp.ad_tool = 'casadi-3.5.5'; % 'symbolic-math-tbx'; %'casadi351';
    codeoptions.solver_timeout = 1; % Setting the option to 1 will enable the timeout and provide the floating point variable solver_timeout as a runtime parameter. Setting the option to 2 will additionally provide the floating point variable timeout_estimate_coeff as a runtime parameter.
    codeoptions.printlevel = 0; % on some platforms printing is not supported
    codeoptions.cleanup = 0; % to keep necessary files for targe compiling
    codeoptions.overwrite = 1;
    % codeoptions.warmstart = 1;
    codeoptions.sse = 1; % SIMD usage
%     codeoptions.noVariableElimination = 1; 
    codeoptions.nlp.checkFunctions = 0;
    codeoptions.solvemethod = 'PDIP_NLP';

%     codeoptions.nlp.TolStat = 5e-1; % inf norm tol. on stationarity
%     codeoptions.nlp.TolEq = 1e-1; % tol. on equality constraints
%     codeoptions.nlp.TolIneq = 1e-1; % tol. on inequality constraints
%     codeoptions.nlp.TolComp = 1e-1; % tol. on complementarity
% 
%     codeoptions.accuracy.ineq = 1e-1; % infinity norm of residual for inequalities
%     codeoptions.accuracy.eq = 1e-1; % infinity norm of residual for equalities
%     codeoptions.accuracy.mu = 1e-1; % absolute duality gap
%     codeoptions.accuracy.rdgap = 5e-1; % relative duality gap := (pobj-dobj)/pobj

%     codeoptions.nlp.TolStat = 1e-5; % inf norm tol. on stationarity
%     codeoptions.nlp.TolEq = 1e-6; % tol. on equality constraints
%     codeoptions.nlp.TolIneq = 1e-6; % tol. on inequality constraints
%     codeoptions.nlp.TolComp = 1e-6; % tol. on complementarity
% 
%     codeoptions.accuracy.ineq = 1e-6; % infinity norm of residual for inequalities
%     codeoptions.accuracy.eq = 1e-6; % infinity norm of residual for equalities
%     codeoptions.accuracy.mu = 1e-6; % absolute duality gap
%     codeoptions.accuracy.rdgap = 1e-4; % relative duality gap := (pobj-dobj)/pobj

%     codeoptions.nlp.TolStat = 1e-1; % inf norm tol. on stationarity
%     codeoptions.nlp.TolEq = 1e-2; % tol. on equality constraints
%     codeoptions.nlp.TolIneq = 1e-2; % tol. on inequality constraints
%     codeoptions.nlp.TolComp = 1e-2; % tol. on complementarity
% 
%     codeoptions.accuracy.ineq = 1e-2; % infinity norm of residual for inequalities
%     codeoptions.accuracy.eq = 1e-2; % infinity norm of residual for equalities
%     codeoptions.accuracy.mu = 1e-2; % absolute duality gap
%     codeoptions.accuracy.rdgap = 1e-1; % relative duality gap := (pobj-dobj)/pobj

    if store_mex_data
        codeoptions.MEXinterface.dynamics = 1;
        codeoptions.MEXinterface.inequalities = 1;
        codeoptions.MEXinterface.objective = 1;
    end

    
    % Generate solver
    FORCES_NLP(model, codeoptions, outputs);

end

%% AUXILIARY FUNCTIONS

%  Objetive functions
% Initial
function f = objectiveInitial(z,p)

    % Parameters
    beta_steer = p(6);
    beta_throttle = p(33);
    beta_epsilon = p(46);

    % Inputs
    d_throttle = z(1);
    d_steering = z(2);
    epsilon = z(3);


    f = beta_steer*d_steering^2 + beta_throttle*d_throttle^2 + beta_epsilon * epsilon^2; 
end
% Middle
function f = objectiveMiddle(z,p)
    global integrator_stepsize;

    alpha_CL = p(1);
    alpha_L = p(2);
    q_r = p(3);
    beta_steer = p(6);
    beta_throttle = p(33);
    lambda = p(7);
    v_max = p(8);
    q_v_max = p(9);
    mu_x = p(35);
    mu_y = p(36);
    g = p(21);
    beta_epsilon = p(46);


    % Derivatives
    d_throttle = z(1);
    d_steering = z(2);
    % Slack
    epsilon = z(3);
    % Progress
    s = z(11);
    % States
    X = z(12);
    Y = z(13);
    yaw = z(14);
    vx = z(15);
    vy = z(16);
    r = z(17);

    [g_x, g_y, dg_x, dg_y] = splineTrack(s,p);

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;

    dg = [dg_x; dg_y];
    v = [vx;vy];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    vk = dg'*R*v;
    
    e_CL = dg_y*(X - g_x) - dg_x*(Y - g_y);
    e_L = -dg_x*(X - g_x) - dg_y*(Y - g_y);

    vel = sqrt(vx^2+vy^2);

    f = alpha_CL*e_CL^2 + alpha_L*e_L^2 + beta_steer*d_steering^2 + beta_throttle*d_throttle^2 + exp(q_v_max*(vel-v_max)) - lambda*vk*integrator_stepsize + beta_epsilon * epsilon^2;
end
%N-1
function f = objectiveN_1(z,p)
    global integrator_stepsize;

   % Parameters
    alpha_CL = p(1);
    alpha_L = p(2);
    q_r = p(3);
    lambda = p(7);
    v_max = p(8);
    q_v_max = p(9);
    mu = p(35);
    g = p(21);
    beta_epsilon = p(46);

    % Slack
    epsilon = z(1);
    % Progress
    s = z(9);
    % States
    X = z(10);
    Y = z(11);
    yaw = z(12);
    vx = z(13);
    vy = z(14);
    r = z(15);

    [g_x, g_y, dg_x, dg_y] = splineTrack(s,p);

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;

    dg = [dg_x; dg_y];
    v = [vx; vy];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    vk = dg'*R*v;
    
    e_CL = dg_y*(X - g_x) - dg_x*(Y - g_y);
    e_L = -dg_x*(X - g_x) - dg_y*(Y - g_y);

    vel = sqrt(vx^2+vy^2);

    f = alpha_CL*e_CL^2 + alpha_L*e_L^2 + exp(q_v_max*(vel-v_max)) - lambda*vk*integrator_stepsize + beta_epsilon * epsilon^2;
end
% End
function f = objectiveLast(z,p)
    global integrator_stepsize;

    % Parameter
    alpha_CL = p(1);
    alpha_L = p(2);
    q_r = p(3);
    lambda = p(7);
    v_max = p(8);
    q_v_max = p(9);
    mu = p(35);
    g = p(21);
    beta_epsilon = p(46);

    % Slack
    epsilon = z(1);
    % Progress
    s = z(5);
    % States
    X = z(6);
    Y = z(7);
    yaw = z(8);
    vx = z(9);
    vy = z(10);
    r = z(11);

    [g_x, g_y, dg_x, dg_y] = splineTrack(s,p);

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;

    dg = [dg_x; dg_y];
    v = [vx; vy];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    vk = dg'*R*v;
    
    e_CL = dg_y*(X - g_x) - dg_x*(Y - g_y);
    e_L = -dg_x*(X - g_x) - dg_y*(Y - g_y);

    vel = sqrt(vx^2+vy^2);

    f = alpha_CL*e_CL^2 + alpha_L*e_L^2 + exp(q_v_max*(vel-v_max)) - lambda*vk*integrator_stepsize + beta_epsilon * epsilon^2;
end

% Model Dynamics
% Progress Dynamics
function progress = progress_dynamics(s,yaw,vx,vy,p)
    global integrator_stepsize;

    [~, ~, dg_x, dg_y] = splineTrack(s,p);
    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;
    dg = [dg_x; dg_y];
    v = [vx; vy];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];

    vk = dg'*R*v;
    progress = s + vk*integrator_stepsize;
end
% Blended Bicicle Model
function blended = vehicle_dynamics(x,u,p)
    global integrator_stepsize;
    lambda_blend_min = p(24);
    lambda_blend_max = p(25);
    l_r = p(11);
    l_f = p(10);

    % Velocity
    velocity = sqrt(x(4)^2+x(5)^2);

    % discritized vehicle model
    dyn_bike = ForwardEuler(x,u,@dynamic_bicycle_model, integrator_stepsize,p);

    %Discritized Kinematic Vehicle Model
    kin_bike = ForwardEuler(x,u,@kinematic_bicycle_model, integrator_stepsize,p);
    Beta = atan(l_r/(l_f+l_r)*tan(u(2)));
    kin_bike(6) = velocity/l_r*sin(Beta);   
    
    % blend the two models
    lambda_ = ForcesMin(ForcesMax((velocity - lambda_blend_min)/(lambda_blend_max - lambda_blend_min), 0), 1);
    
    blended = lambda_*dyn_bike + (1-lambda_)*kin_bike;
end
% Dynamic Bicycle Model
function dX = dynamic_bicycle_model(x,u,p)
    % x = [1 - X
    %      2 - Y
    %      3 - Psi
    %      4 - vx
    %      5 - vy
    %      6 - r]

    % u = [1 - d 
    %      2 - delta
    %      3 - d_prev
    %      4 - delta_prev]

    % Parameters
%     l_f = p(10);
%     l_r = p(11);
%     g = p(21);
%     m = p(12);
%     rho = p(23);
%     C_l = p(27);
%     downforce_front = p(31);
%     downforce_rear = p(32);
%     T_max_front = p(14);
%     T_max_rear = p(15);
%     T_brake_front = p(16);
%     T_brake_rear = p(17);
%     GR = p(18);
%     r_wheel = p(20);
%     eta_motor = p(19);
%     C_roll = p(22);
%     C_d = p(26);
%     D = p(30);
%     C = p(29);
%     B = p(28);
%     I_z = p(13);
%     h_cog = p(34);
%     NN_flag = p(49);
%     track_width = p(52);
%     diff_gain = p(53);
% 
%     % Rolling Resistance
%     F_roll = C_roll*m*g;
%     
%     % Drag Force
%     F_drag = 1/2*rho*C_d*x(4)^2;
%     
%     % Braking or Accel Torque
%     t_bool = (u(1) < 0);
%     T_front = ((~t_bool)*T_max_front + t_bool*(T_brake_front)) * u(1);
%     T_rear = ((~t_bool)*T_max_rear + t_bool*(T_brake_rear)) * u(1);
% 
%     % Longitudinal Load Transfer
%     F_f_ax = 2*eta_motor*T_front*GR/r_wheel*u(3);
%     F_r_ax = 2*eta_motor*T_rear*GR/r_wheel*u(3);
%     a_x = (F_f_ax*cos(u(4)) + F_r_ax - F_roll - F_drag)/m + x(5)*x(6);
%     delta_Fz_x = h_cog/(l_r+l_f)*m*a_x;
% 
% %     a_x = get_ax(x(4),p(end-2),x(5),x(6));
% %     delta_Fz_x = h_cog/(l_r+l_f)*m*a_x/2;
% 
%     % Differential
%     [diff_front, diff_rear] = differential(x(4),u(2),a_x,diff_gain);
% 
%     % Saturate
%     T_fl = ForcesMax(ForcesMin(T_front + diff_front/2,T_max_front),-T_brake_front);
%     T_fr = ForcesMax(ForcesMin(T_front - diff_front/2,T_max_front),-T_brake_front);
%     T_rl = ForcesMax(ForcesMin(T_rear + diff_rear/2,T_max_rear),-T_brake_rear);
%     T_rr = ForcesMax(ForcesMin(T_rear - diff_rear/2,T_max_rear),-T_brake_rear);
%     
%     % Get longitudinal Forces
%     Fx_fl = eta_motor*T_fl*GR/r_wheel;
%     Fx_fr = eta_motor*T_fr*GR/r_wheel;
%     Fx_rl = eta_motor*T_rl*GR/r_wheel;
%     Fx_rr = eta_motor*T_rr*GR/r_wheel;
% 
%     % Fet differential Moment
%     diff_Mz = track_width/2*((Fx_fr-Fx_fl)*cos(u(2))+(Fx_rr-Fx_rl));
% 
%     % Static weight distribution
%     Ff_z_static = m * (l_r/(l_f + l_r)) * g;
%     Fr_z_static = m * (l_f/(l_f + l_r)) * g;
% 
%     % Downforce Distribution
%     F_downforce = 0.5 * rho * C_l * x(4)^2;
% 
%     Ff_downforce = F_downforce * downforce_front;
%     Fr_downforce = F_downforce * downforce_rear;
% 
%     % Slip Angles
% %     alpha_f = u(2) - atan2( (x(5) + x(6)*l_f) , x(4) );
% %     alpha_r = atan2( (x(5) - x(6)*l_r) , x(4) );
%     alpha_f = u(2) - (x(5) + x(6)*l_f) / (x(4));
%     alpha_r = (x(5) - x(6)*l_r) / (x(4));
% 
%     % Static + Downforce + Longitudinal Load Transfer
%     Fz_f = Ff_z_static/2 + Ff_downforce - delta_Fz_x;
%     Fz_r = Fr_z_static/2 + Fr_downforce + delta_Fz_x;
% 
%     % Lateral Forces
%     Ff_tire = Fz_f*D*sin(C*atan(B*alpha_f));
%     Fr_tire = Fz_r*D*sin(C*atan(B*alpha_r));
% 
%     % Longitudinal Forces
%     F_f = Fx_fl + Fx_fr;
%     F_r = Fx_rl + Fx_rr;
%     
%     % Longitudinal and Lateral Force and Yaw Moment
%     F_x = F_f*cos(u(2)) - Ff_tire*sin(u(2)) + F_r - F_roll - F_drag;
%     F_y = F_f*sin(u(2)) + Ff_tire*cos(u(2)) - Fr_tire;
%     M_z = (F_f*sin(u(2)) + Ff_tire*cos(u(2)))*l_f + Fr_tire*l_r + diff_Mz;
% 
%     % NN correction
%     NN_input = [u(3) u(4) p(end-2) p(end-1) p(end); u(1) u(2) x(4) x(5) x(6)];
%     NN_correction = NN_flag * NNmodel(NN_input,p(1:end-3));
% 
%     % State derivative function
%     cos_Phi = cos(x(3));
%     sin_Phi = sin(x(3));
%     dX = [x(4)*cos_Phi - x(5)*sin_Phi;
%           x(4)*sin_Phi + x(5)*cos_Phi;
%           x(6);
%           F_x/m + x(5)*x(6) + NN_correction(1);
%           F_y/m - x(4)*x(6) + NN_correction(2);
%           M_z/I_z + NN_correction(3)];


    % Parameters
    l_f = p(10);
    l_r = p(11);
    g = p(21);
    m = p(12);
    rho = p(23);
    C_l = p(27);
    downforce_front = p(31);
    downforce_rear = p(32);
    T_max_front = p(14);
    T_max_rear = p(15);
    T_brake_front = p(16);
    T_brake_rear = p(17);
    GR = p(18);
    r_wheel = p(20);
    eta_motor = p(19);
    C_roll = p(22);
    C_d = p(26);
    D = p(30);
    C = p(29);
    B = p(28);
    I_z = p(13);
    h_cog = p(34);
    NN_flag = p(49);
    track_width = p(52);
    diff_gain = p(53);

    % Rolling Resistance
    F_roll = C_roll*m*g;
    
    % Drag Force
    F_drag = 1/2*rho*C_d*x(4)^2;
    
    % Braking or Accel Torque
    t_bool = (u(1) < 0);
    T_front = ((~t_bool)*T_max_front + t_bool*(T_brake_front))*u(1);
    T_rear = ((~t_bool)*T_max_rear + t_bool*(T_brake_rear))*u(1);

    % Longitudinal Load Transfer
    a_x = get_ax(x(4),p(end-2),x(5),x(6));
    delta_Fz_x = h_cog/(l_r+l_f)*m*a_x;

    % Differential
    [diff_front, diff_rear] = differential(x(4),u(2),a_x,diff_gain);

    % Saturate
    T_fl = ForcesMax(ForcesMin(T_front + diff_front/2,T_max_front),-T_brake_front);
    T_fr = ForcesMax(ForcesMin(T_front - diff_front/2,T_max_front),-T_brake_front);
    T_rl = ForcesMax(ForcesMin(T_rear + diff_rear/2,T_max_rear),-T_brake_rear);
    T_rr = ForcesMax(ForcesMin(T_rear - diff_rear/2,T_max_rear),-T_brake_rear);
    
    % Get longitudinal Forces
    F_fl = eta_motor*T_fl*GR/r_wheel;
    F_fr = eta_motor*T_fr*GR/r_wheel;
    F_rl = eta_motor*T_rl*GR/r_wheel;
    F_rr = eta_motor*T_rr*GR/r_wheel;

    % Fet differential Moment
    diff_Mz = track_width/2*((F_fr-F_fl)*cos(u(2))+(F_rr-F_rl));

    % Static weight distribution
    Ff_z_static = m * (l_r/(l_f + l_r)) * g;
    Fr_z_static = m * (l_f/(l_f + l_r)) * g;

    % Downforce Distribution
    F_downforce = 0.5 * rho * C_l * x(4)^2;

    Ff_downforce = F_downforce * downforce_front;
    Fr_downforce = F_downforce * downforce_rear;

    % Slip Angles vx+1.5./(1+vx))
    alpha_f = u(2) - atan2( x(5) + x(6)*l_f , x(4) );
    alpha_r = atan2(x(5) - x(6)*l_r , x(4) );

    % Lateral Load Transfer
%     a_y = x(4)*x(6);
%     delta_Fz_y = h_cog/track_width*m*a_y;

    % Static + Downforce + Longitudinal Load Transfer
    Ff_z = Ff_z_static + Ff_downforce - delta_Fz_x;
    Fr_z = Fr_z_static + Fr_downforce + delta_Fz_x;

    % Lateral Forces
    Ff_tire = Ff_z*D*sin(C*atan(B*alpha_f));
    Fr_tire = Fr_z*D*sin(C*atan(B*alpha_r));
   
    % Longitudinal Forces
    F_f = F_fl + F_fr;
    F_r = F_rl + F_rr;
    
    % Longitudinal and Lateral Force and Yaw Moment
    F_x = F_f*cos(u(2)) - Ff_tire*sin(u(2)) + F_r - F_roll - F_drag;
    F_y = F_f*sin(u(2)) + Ff_tire*cos(u(2)) - Fr_tire;
    M_z = (F_f*sin(u(2)) + Ff_tire*cos(u(2)))*l_f + Fr_tire*l_r + diff_Mz;
    
    % State derivative function
    cos_Phi = cos(x(3));
    sin_Phi = sin(x(3));
    dX = [x(4)*cos_Phi - x(5)*sin_Phi;
          x(4)*sin_Phi + x(5)*cos_Phi;
          x(6);
          F_x/m + x(5)*x(6);
          F_y/m - x(4)*x(6);
          M_z/I_z];
end
% Kinematic Bicycle Model
function dX = kinematic_bicycle_model(x,u,p)
    % x = [1 - X
    %      2 - Y
    %      3 - Psi
    %      4 - vx
    %      5 - vy
    %      6 - r]

    % u = [1 - d 
    %      2 - delta]

    % Parameters
    l_f = p(10);
    l_r = p(11);
    m = p(12);
    T_max_front = p(14);
    T_max_rear = p(15);
    T_brake_front = p(16);
    T_brake_rear = p(17);
    GR = p(18);
    r_wheel = p(20);
    eta_motor = p(19);

    % Side Slip Angle
    Beta = atan(l_r/(l_f+l_r)*tan(u(2)));

    % Braking or Accel Torque
    t_bool = (u(1) < 0);
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front);
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear);
    
    % Front and rear forces
    F_f = 2*eta_motor*T_front*GR/r_wheel*u(1);
    F_r = 2*eta_motor*T_rear*GR/r_wheel*u(1);

    % Fx and Fy
    F_x = F_f*cos(u(2)) + F_r;
    F_y = F_f*sin(u(2));

    % Velocity
    vel = sqrt(x(4)^2 + x(5)^2);

    % State derivative function
    dX = [vel*cos(x(3)+Beta);...
          vel*sin(x(3)+Beta);...
          vel/l_r*sin(Beta);...
          F_x/m;
          F_y/m;
          0];

end

% Nonlinear Inequalities
function h = ineqI(z,p)    
    % Parameters
    delta_throttle_max = p(41);
    delta_steering_max = p(42);
    mu_x = p(35);
    mu_y = p(36);
    g = p(21);

    d_throttle = z(1);
    d_steering = z(2);
    epsilon = z(3);
    vx = z(15);
    vx_prev = z(8);
    vy = z(16);
    vy_prev = z(9);
    r = z(17);

    % ax
    a_x = get_ax(vx,vx_prev,vy,r);
    % ay
    a_y = get_ay(vy,vy_prev,vx,r);

    h = [
         d_throttle/delta_throttle_max;       % rate throttle
         d_steering/delta_steering_max;       % rate steering
         (a_x/mu_x)^2 + (a_y/mu_y)^2 - (g)^2 - epsilon       % friction circle
         ];   
end

function h = ineqM(z,p)
    % Parameters
    delta_throttle_max = p(41);
    delta_steering_max = p(42);
    g = p(21);
    m = p(12);
    rho = p(23);
    C_roll = p(22);
    C_d = p(26);
    C_l = p(27);
    mu_x = p(35);
    mu_y = p(36);

    % Derivatives
    d_throttle = z(1);
    d_steering = z(2);
    % Slack
    epsilon = z(3);
    % Previous inputs
    throttle_prev = z(6);
    steering_prev = z(7);
    % Previous States
    vx_prev = z(8);
    vy_prev = z(9);
    r_prev = z(10);
    % States
    vx = z(15);
    vy = z(16);
    r = z(17);
    
    % ax
%     accel_downforce = (0.5 * rho * C_l * vx^2)/m;
    a_x = get_ax(vx,vx_prev,vy,r);
    % ay
    a_y = get_ay(vy,vy_prev,vx,r);

    h = [
         d_throttle/delta_throttle_max;       % rate throttle
         d_steering/delta_steering_max;       % rate steering
         (a_x/mu_x)^2 + (a_y/mu_y)^2 - (g)^2 - epsilon       % friction circle
         ];  
end

function h = ineqN_1(z,p)
    % Parameters
    g = p(21);
    m = p(12);
    rho = p(23);
    C_roll = p(22);
    C_d = p(26);
    C_l = p(27);
    mu_x = p(35);
    mu_y = p(36);

    % Slack
    epsilon = z(1);
    % Previous inputs
    throttle_prev = z(4);
    steering_prev = z(5);
    % Previous States
    vx_prev = z(6);
    vy_prev = z(7);
    r_prev = z(8);
    % States
    vx = z(13);
    vy = z(14);
    r = z(15);
    
    % ax
    a_x = get_ax(vx,vx_prev,vy,r);
    % ay
    a_y = get_ay(vy,vy_prev,vx,r);

    h = [
         (a_x/mu_x)^2 + (a_y/mu_y)^2 - (g)^2 - epsilon       % friction circle
        ];
end

function h = ineqN(z,p)
    % Parameters
    mu_x = p(35);
    mu_y = p(36);
    g = p(21);

    % Slack
    epsilon = z(1);
    % Previous States
    vx_prev = z(2);
    vy_prev = z(3);
    r_prev = z(4);
    % States
    vx = z(9);
    vy = z(10);
    r = z(11);

    % ax
    a_x = get_ax(vx,vx_prev,vy,r);
    % ay
    a_y = get_ay(vy,vy_prev,vx,r);

    h = [
        (a_x/mu_x)^2 + (a_y/mu_y)^2 - (g)^2 - epsilon       % friction circle
        ];
end

function [x,y,dx,dy] = splineTrack(s,p)
    global config_data
    path_length = p(54);
    n_points = config_data.mpc_params.objective.spline;
    parameter_offset = config_data.mpc_params.objective.p + config_data.NN_config.NN_params; % Number of parameters before spline parameters

    coefs_mul = [1,2,3];
    coefs_mul = ones(n_points-1,3).*coefs_mul;
    % X splines
    breaks = linspace(0,path_length,n_points);

    coefs_x = casadi.SX(n_points-1,4);
    for i =1:n_points-1
        coefs_x(i,:) = p(parameter_offset+(i-1)*4+1:parameter_offset+(i-1)*4+4);
    end
    coefs_x = flip(coefs_x,2);
    xSpline = ForcesInterpolation(breaks,coefs_x,'pp');

    % dX splines
    coefs_dx = casadi.SX(n_points-1,3);
    for i = 1:n_points-1
        coefs_dx(i,:) = p(parameter_offset+(i-1)*4+2:parameter_offset+(i-1)*4+4);
    end
    coefs_dx = coefs_dx.*coefs_mul;
    coefs_dx = flip(coefs_dx,2);
    dxSpline = ForcesInterpolation(breaks,coefs_dx,'pp');

    % Y splines
    coefs_y = casadi.SX(n_points-1,4);
    for i =1:n_points-1
        coefs_y(i,:) = p(parameter_offset+(n_points-1)*4+(i-1)*4+1:parameter_offset+(n_points-1)*4+(i-1)*4+4);
    end
    coefs_y = flip(coefs_y,2);
    ySpline = ForcesInterpolation(breaks,coefs_y,'pp');

    %dY splines
    coefs_dy = casadi.SX(n_points-1,3);
    for i = 1:n_points-1
        coefs_dy(i,:) = p(parameter_offset+(n_points-1)*4+(i-1)*4+2:parameter_offset+(i-1)*4+(n_points-1)*4+4);
    end
    coefs_dy = coefs_dy.*coefs_mul;
    coefs_dy = flip(coefs_dy,2);

    dySpline = ForcesInterpolation(breaks,coefs_dy,'pp');

    % Evaluate
    progress = casadi.SX(s);

    x = xSpline(progress);
    y = ySpline(progress);
    dx = dxSpline(progress);
    dy = dySpline(progress);
end

function y=euler(x,dx)
    global integrator_stepsize
    y = x + dx*integrator_stepsize;
end

function out = NNmodel(x,p) % Linear
    global config_data
    in0 = x(1,:)'; 
    in1 = x(2,:)';

    ind = config_data.mpc_params.objective.p + 1; % CHANGE THIS PARAMETER
    % FC1
    NNparams.fc1_kernel = reshape(p(ind:ind+(16*5)-1),[16,5]);
    ind = ind + 16*5;
    NNparams.fc1_bias = reshape(p(ind:ind+16-1),[16,1]);
    ind = ind + 16;
    % FC2
    NNparams.fc2_kernel = reshape(p(ind:ind+(8*16)-1),[8,16]);
    ind = ind + 8*16;
    NNparams.fc2_bias = reshape(p(ind:ind+8-1),[8,1]);
    ind = ind + 8;
    % RNN
    NNparams.rnn_kernel = reshape(p(ind:ind+(8*8)-1),[8,8]);
    ind = ind + 8*8;
    NNparams.rnn_recurrent_kernel = reshape(p(ind:ind+(8*8)-1),[8,8]);
    ind = ind + 8*8;
    NNparams.rnn_bias = reshape(p(ind:ind+8-1),[8,1]);
    ind = ind + 8;
    NNparams.rnn_recurrent_bias = reshape(p(ind:ind+8-1),[8,1]);
    ind = ind + 8;
    % FC2_REV
    NNparams.fc2_rev_kernel = reshape(p(ind:ind+(8*16)-1),[16,8]);
    ind = ind + 16*8;
    NNparams.fc2_rev_bias = reshape(p(ind:ind+16-1),[16,1]);
    ind = ind + 16;
    % FC1_REV
    NNparams.fc1_rev_kernel = reshape(p(ind:ind+(5*16)-1),[5,16]);
    ind = ind + 5*16;
    NNparams.fc1_rev_bias = reshape(p(ind:ind+5-1),[5,1]);
    ind = ind + 5;
    % FC_OUT
    NNparams.fc_out_kernel = reshape(p(ind:ind+(3*5)-1),[3,5]);
    ind = ind + 3*5;
    NNparams.fc_out_bias = reshape(p(ind:ind+3-1),[3,1]);

    % Expand space
    x0 = GELU(NNparams.fc1_kernel*in0 + NNparams.fc1_bias);
    x0 = NNparams.fc2_kernel*x0 + NNparams.fc2_bias;

    x1 = GELU(NNparams.fc1_kernel*in1 + NNparams.fc1_bias);
    x1 = NNparams.fc2_kernel*x1 + NNparams.fc2_bias;
    
    % RNN
    h0 = [0;0;0;0;0;0;0;0];
    h1 = tanh(NNparams.rnn_kernel*x0 + NNparams.rnn_recurrent_kernel*h0 + NNparams.rnn_bias + NNparams.rnn_recurrent_bias);
    h2 = tanh(NNparams.rnn_kernel*x1 + NNparams.rnn_recurrent_kernel*h1 + NNparams.rnn_bias + NNparams.rnn_recurrent_bias);
    
    % Compress and output
    out = GELU(NNparams.fc2_rev_kernel*h2 + NNparams.fc2_rev_bias);
    out = NNparams.fc1_rev_kernel*out + NNparams.fc1_rev_bias;
    
    out = NNparams.fc_out_kernel*out + NNparams.fc_out_bias;
end

function Y=GELU(X)
    Y = X.*0.5.*(1+erf(X./sqrt(2)));
end

function [diff_front, diff_rear] = differential(v_x,delta,a_x,diff_gain)
    global diff_Coeffs;
    ratio = ForcesMax(ForcesMin(a_x/9.81,1),-1)*0.16;

    diff_T = diff_gain*(diff_Coeffs(1)*delta + diff_Coeffs(2)*v_x*delta + diff_Coeffs(3)*delta^3);

    diff_front = diff_T*(0.4-ratio);
    diff_rear = diff_T*(0.6+ratio);
end

function a_x = get_ax(vx,vx_prev, vy, r)
    global integrator_stepsize
    a_x = (vx-vx_prev)/integrator_stepsize - vy*r;
end

function a_y = get_ay(vy,vy_prev,vx,r)
    global integrator_stepsize
    a_y = (vy-vy_prev)/integrator_stepsize + vx*r;
end
%% ----------- Exit Flags -----------------------------------------------
%  2 The solver timed out and returned the solution found up to the executed iteration
%  1 Local optimal solution found (i.e. the point satisfies the KKT optimality conditions to the requested accuracy).
%  0 Maximum number of iterations reached. You can examine the value of optimality conditions returned inside the info struct by FORCESPRO to decide
% whether the point returned is acceptable
% -4 Wrong number of inequalities input to solver.
% -5 Error occured during matrix factorization.
% -6 NaN or INF occurred during functions evaluations.
% -7 The solver could not proceed. Most likely cause is that the problem is infeasible.Try formulating a problem with slack variables (soft constraints) to avoid
% this error.
% -8 The internal QP solver could not proceed. This exitflag can only occur when using the Sequential quadratic programming algorithm. The most likely cause
% is that an infeasible QP or a numerical unstable QP was encountered. Try increasing the hessian regularization parameter reg_hessian if this exitflag is
% encountered (see SQP specific codeoptions).
% -10 NaN or INF occurred during evaluation of functions and derivatives. If this occurs at iteration zero, try changing the initial point. For example, for a cost function 1/
% √𝑥 with an initialization 𝑥0 = 0, this error would occur.
% -11 Invalid values in problem parameters.
% -12 The timeout provided was too small to even start a single iteration
% -100 License error. This typically happens if you are trying to execute code that has
% been generated with a Simulation license of FORCESPRO on another machine.
% Regenerate the solver using your machine
% -101 insufficient max_num_mem
% -102 insufficient max_num_threads