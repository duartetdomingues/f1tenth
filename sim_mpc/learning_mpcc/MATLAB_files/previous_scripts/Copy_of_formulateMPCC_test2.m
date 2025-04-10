function formulateMPCC_test2(matSpline,centerline) %TO DO - ADD ELLIPSE CONSTRAINT, add cost term regarding the slip angle in order to reduce aggressiveness (AMZ), use the derivative of the control input, incorporate lift force(downforce), add cone related constraints, look into the warm start, check for inequalities that depend on consecutive timesteps, might be necessary to take the last ones, tire models need a closer look, add the jacobian (might speed up), experiment with different solvers to see which one performs better, decouple the throttle from the problem and send that command to a PID controller like pure pursuit did
    global config_data;
    global track_length;
    global EXTEND_CENTERLINE;

    %% Create Splines
    global xSpline_FORCES;
    global ySpline_FORCES;
    global dxSpline_FORCES;
    global dySpline_FORCES;

    % Conversion for ForcesPro
    xSpline_FORCES = ForcesInterpolation(matSpline.xSpline);
    ySpline_FORCES = ForcesInterpolation(matSpline.ySpline);
    dxSpline_FORCES = ForcesInterpolation(matSpline.dxSpline);
    dySpline_FORCES = ForcesInterpolation(matSpline.dySpline);

    %% Plot data - See if splines are well computed
    figure
    % centerline
    plot(centerline.X,centerline.Y,'--');
    hold on
    % spline
    plot(xSpline_FORCES(0:track_length),ySpline_FORCES(0:track_length));
    % spline derivative
    x = xSpline_FORCES(linspace(0,track_length,30));
    y = ySpline_FORCES(linspace(0,track_length,30));
    P1 = [x' y'];
    dx = dxSpline_FORCES(linspace(0,track_length,30));
    dy = dySpline_FORCES(linspace(0,track_length,30));
    P2 = [(x+2*dx)' (y+2*dy)'];
    arrow3(P1,P2,'k',0.25,0.25,0,1,0.4);
    hold off

    %% Read Tunable Parameters from File and Store

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

    %% FORMULATION

    model = {};

    N = config_data.mpc_params.objective.N;
    model.N = N;
    global integrator_stepsize
    integrator_stepsize = 1/config_data.controller_freq;

    n_state = 6; % X, Y, Psi, vx, vy, r
    additional_variables = 4; % d(t-1), delta(t-1), s(t-1), s(t)
    n_input = 2; % throttle, steering

    model.npar = 32;

    % p =  [1 - alpha_CL
    %       2 - alpha_L
    %       3 - q_v_y 
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
    %       32 - downforce_rear]

    %% First timestep

    % z =  [1 - d(t)
    %       2 - delta(t)
    %       3 - d(t-1)
    %       4 - delta(t-1)
    %       5 - s(t)
    %       6 - X 
    %       7 - Y 
    %       8 - Psi 
    %       9 - vx 
    %       10 - vy 
    %       11 - r]

    % Problem dimensions
    model.nvar(1) = n_input + additional_variables - 1 + n_state; % input + all additional states minus last progress + states
    model.xinitidx = 3:11;
    model.neq(1) =  additional_variables + n_state;
    model.nh(1) = 2; %input rate constraints

    % Objective function
    model.objective{1} = @objectiveInitial;

    % State Transition
    model.eq{1} = @(z,p) [z(1); z(2); z(5); progress_dynamics(z(5),z(8),z(9),z(10)); vehicle_dynamics(z(6:11),z(1:2),p)];
    model.E{1} = [zeros(model.neq(1),n_input) eye(model.neq(1))];

    % Simple Inequalities
    %             [d(t)          delta(t)      d(t-1)   delta(t-1)  s(t)                              X     Y     Psi   vx    vy    r   ]
    model.lb{1} = [throttle_min, steering_min, -inf,    -inf,       0,                                -inf, -inf, -inf, 0   , -inf, -inf];
    model.ub{1} = [throttle_max, steering_max, +inf,    +inf,       track_length+EXTEND_CENTERLINE ,  +inf, +inf, +inf, +inf, +inf, +inf];

    % Inequalities
    model.ineq{1} = @ineqI;
    model.hl{1} = [delta_throttle_min, delta_steering_min];
    model.hu{1} = [delta_throttle_max, delta_steering_max];

    %% In between timesteps

    % z =  [1 - d(t) 
    %       2 - delta(t)
    %       3 - d(t-1) 
    %       4 - delta(t-1)
    %       5 - s(t-1)
    %       6 - s(t)
    %       7 - X 
    %       8 - Y 
    %       9 - Psi 
    %       10 - vx 
    %       11 - vy 
    %       12 - r]

    for i=2:N-2
        % Problem dimensions
        model.nvar(i) = n_input + additional_variables + n_state;
        model.neq(i) =  additional_variables + n_state;
        model.nh(i) = 4; %e_CL constraint, input rate constraints, progress rate
    
        % Objective function
        model.objective{i} = @objectiveMiddle;
    
        % State Transition
        model.eq{i} = @(z,p) [z(1); z(2); z(6); progress_dynamics(z(6),z(9),z(10),z(11)); vehicle_dynamics(z(7:12),z(1:2),p)];
        model.E{i} = [zeros(model.neq(i),n_input) eye(model.neq(i))];
    
        % Simple Inequalities
        %             [d(t)          delta(t)      d(t-1)   delta(t-1)   s(t-1)   s(t)                            X     Y     Psi   vx    vy    r   ]
        model.lb{i} = [throttle_min, steering_min, -inf,    -inf,        -inf,    0,                              -inf, -inf, -inf, 0   , -inf, -inf];
        model.ub{i} = [throttle_max, steering_max, +inf,    +inf,        +inf,    track_length+EXTEND_CENTERLINE, +inf, +inf, +inf, +inf, +inf, +inf];
    
        % Inequalities
        model.ineq{i} = @ineqM;
        model.hl{i} = [e_CL_min, delta_throttle_min, delta_steering_min, delta_s_min];
        model.hu{i} = [e_CL_max, delta_throttle_max, delta_steering_max, delta_s_max];
    end
    
    %% Penultimate (needed for state transition but basically the same as middle)

    % Problem dimensions
    model.nvar(N-1) = n_input + additional_variables + n_state;
    model.neq(N-1) =  2 + n_state; %progress, progress(t-1), states
    model.nh(N-1) = 4; %e_CL constraint, input rate constraints, progress rate
   
    % Objective function
    model.objective{N-1} = @objectiveMiddle;
   
    % State Transition
    model.eq{N-1} = @(z,p) [z(6); progress_dynamics(z(6),z(9),z(10),z(11)); vehicle_dynamics(z(7:12),z(1:2),p)];
    model.E{N-1} = [eye(model.neq(N-1))];
   
    % Simple Inequalities
    %               [d(t)          delta(t)      d(t-1)   delta(t-1)   s(t-1) s(t)                          X     Y     Psi   vx    vy    r   ]
    model.lb{N-1} = [throttle_min, steering_min, -inf,    -inf,        -inf,  0,                              -inf, -inf, -inf, 0   , -inf, -inf];
    model.ub{N-1} = [throttle_max, steering_max, +inf,    +inf,        +inf,  track_length+EXTEND_CENTERLINE, +inf, +inf, +inf, +inf, +inf, +inf];
   
    % Inequalities
    model.ineq{N-1} = @ineqM;
    model.hl{N-1} = [e_CL_min, delta_throttle_min, delta_steering_min, delta_s_min];
    model.hu{N-1} = [e_CL_max, delta_throttle_max, delta_steering_max, delta_s_max];

    %% Last

    % z =  [1 - s(t-1)
    %       2 - s(t)
    %       3 - X 
    %       4 - Y 
    %       5 - Psi 
    %       6 - vx 
    %       7 - vy 
    %       8 - r]

    % Problem dimensions
    model.nvar(N) = n_state + 2; % states + progress + progress(t-1)
    model.nh(N) = 2; %e_CL constraint + progress contsraint

    % Objective function
    model.objective{N} = @objectiveLast;
%     model.objective{N} =@(z) 0;


    % Simple Inequalities
    %             [s(t-1)   s(t)                             X    Y     Psi    vx   vy    r   ]
    model.lb{N} = [-inf,    0,                              -inf, -inf, -inf, 0   , -inf, -inf];
    model.ub{N} = [+inf,    track_length+EXTEND_CENTERLINE, +inf, +inf, +inf, +inf, +inf, +inf];

    % Inequalities
    model.ineq{N} = @ineqN;
    model.hl{N} = [e_CL_min, delta_s_min];
    model.hu{N} = [e_CL_max, delta_s_max];



    %% Generate Solver 
    global STORE_MEX_DATA

    % Make the output a continuous vector
    outputs(1) = newOutput('zI',1,1:model.nvar(1));
    outputs(2) = newOutput('z',2:(model.N-1),1:model.nvar(2));
    outputs(3) = newOutput('zN',model.N,1:model.nvar(N));

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
    codeoptions.printlevel = 0; % on some platforms printing is not supported
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
%     codeoptions.noVariableElimination = 1;
    codeoptions.nlp.checkFunctions = 1;
    if STORE_MEX_DATA
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

    alpha_CL = p(1);
    alpha_L = p(2);
    e_CL_exp = p(4);
    beta_steer = p(6);

    [g_x, g_y, dg_x, dg_y] = splineTrack(z(5));

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;

%     splinePsi = atan(dg_y/dg_x);
% 
%     e_CL = sin(0)*(z(6) - g_x) - cos(0)*(z(7) - g_y);
%     e_L = -cos(0)*(z(6) - g_x) - sin(0)*(z(7) - g_y);
    
    e_CL = dg_y*(z(6) - g_x) - dg_x*(z(7) - g_y);
    e_L = -dg_x*(z(6) - g_x) - dg_y*(z(7) - g_y);

    f = alpha_CL*e_CL^e_CL_exp + alpha_L*e_L^2 + beta_steer*(z(2)-z(4))^2;
end
% Middle
function f = objectiveMiddle(z,p)
    global integrator_stepsize;

    alpha_CL = p(1);
    alpha_L = p(2);
    q_v_y = p(3);
    e_CL_exp = p(4);
    beta_steer = p(6);
    lambda = p(7);
    v_max = p(8);
    q_v_max = p(9);

    [g_x, g_y, dg_x, dg_y] = splineTrack(z(6));

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;

    dg = [dg_x; dg_y];
    vx = z(10);
    vy = z(11);
    yaw = z(9);
    v = [vx; vy];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    vk = dg'*R*v;
    
    e_CL = dg_y*(z(7) - g_x) - dg_x*(z(8) - g_y);
    e_L = -dg_x*(z(7) - g_x) - dg_y*(z(8) - g_y);

%     splinePsi = atan(dg_y/dg_x);

%     e_CL = sin(0)*(z(6) - g_x) - cos(0)*(z(7) - g_y);
%     e_L = -cos(0)*(z(6) - g_x) - sin(0)*(z(7) - g_y);

    f = q_v_y*z(11)^2 + alpha_CL*e_CL^e_CL_exp + alpha_L*e_L^2 + beta_steer*(z(2)-z(4))^2 + exp(q_v_max*(z(10)-v_max)) - lambda*vk*integrator_stepsize;
end
% End
function f = objectiveLast(z,p)
    global integrator_stepsize;

    alpha_CL = p(1);
    alpha_L = p(2);
    q_v_y = p(3);
    e_CL_exp = p(4);
    lambda = p(7);
    v_max = p(8);
    q_v_max = p(9);

    [g_x, g_y, dg_x, dg_y] = splineTrack(z(2));

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;

    dg = [dg_x; dg_y];
    vx = z(6);
    vy = z(7);
    yaw = z(5);
    v = [vx; vy];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    vk = dg'*R*v;
    
    e_CL = dg_y*(z(3) - g_x) - dg_x*(z(4) - g_y);
    e_L = -dg_x*(z(3) - g_x) - dg_y*(z(4) - g_y);

%     splinePsi = atan(dg_y/dg_x);
% 
%     e_CL = sin(0)*(z(6) - g_x) - cos(0)*(z(7) - g_y);
%     e_L = -cos(0)*(z(6) - g_x) - sin(0)*(z(7) - g_y);

    f = q_v_y*z(7)^2 + alpha_CL*e_CL^e_CL_exp + alpha_L*e_L^2 + exp(q_v_max*(z(6)-v_max)) - lambda*vk*integrator_stepsize;
end

% Progress
function progress = progress_dynamics(s,yaw,vx,vy)
    global integrator_stepsize;

    [~, ~, dg_x, dg_y] = splineTrack(s);
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

    % discritized vehicle model
    dyn_bike = RK4(x,u,@dynamic_bicycle_model, integrator_stepsize,p);

    %Discritized Kinematic Vehicle Model
    velocity = sqrt(x(4)^2+x(5)^2);
    kin_states = [x(1) x(2) x(3) velocity]';
    kin_states_next = RK4(kin_states,u,@kinematic_bicycle_model, integrator_stepsize,p);
    Beta = atan(l_r/(l_f+l_r)*tan(u(2)));
    kin_bike_1 = kin_states_next(1);
    kin_bike_2 = kin_states_next(2);
    kin_bike_3 = kin_states_next(3);
    kin_bike_4 = kin_states_next(4)*cos(Beta);
    kin_bike_5 = kin_states_next(4)*sin(Beta);
    kin_bike_6 = kin_bike_4/(l_r+l_f)*tan(u(2));
    kin_bike = [kin_bike_1;kin_bike_2;kin_bike_3;kin_bike_4;kin_bike_5;kin_bike_6];

    
    % blend the two models
    lambda_ = min(max((velocity - lambda_blend_max)/(lambda_blend_max - lambda_blend_min), 0), 1);
    
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
    %      2 - delta]

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

    % Static weight distribution
    Ff_z = m * (l_f/(l_f + l_r)) * g;
    Fr_z = m * (l_r/(l_f + l_r)) * g;

    % Downforce Distribution
    F_downforce = 0.5 * rho * C_l * x(4)^2;

    Ff_downforce = F_downforce * downforce_front;
    Fr_downforce = F_downforce * downforce_rear;

    % Static + Downforce
    Ff_z = Ff_z + Ff_downforce;
    Fr_z = Fr_z + Fr_downforce;

    % Slip Angles and Lateral Forces
    alpha_f = u(2) - atan2(x(5) + x(6)*l_f, x(4));
    alpha_r = -atan2(x(5) - x(6)*l_r, x(4));
    Ff_y = -Ff_z*D*sin(C*atan(B*alpha_f));
    Fr_y = -Fr_z*D*sin(C*atan(B*alpha_r));
    
    % Braking or Accel Torque
    t_bool = (u(1) < 0);
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front);
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear);
   
    F_f = 2*eta_motor*T_front*GR/r_wheel*u(1);
    F_r = 2*eta_motor*T_rear*GR/r_wheel*u(1);
    
    F_roll = C_roll*m*g;
    
    F_drag = 1/2*rho*C_d*x(4)^2;
    
    F_x = F_f*cos(u(2)) + F_r - F_roll - F_drag - Ff_y*sin(u(2));
    F_y = F_f*sin(u(2)) + Ff_y*cos(u(2)) + Fr_y;
    M_z = (F_f*sin(u(2))+Ff_y*cos(u(2)))*l_f - Fr_y*l_r;
    
    % State derivative function
    cos_Phi = cos(x(3));
    sin_Phi = sin(x(3));
    dX = [x(4)*cos_Phi - x(5)*sin_Phi;
          x(4)*sin_Phi + x(5)*cos_Phi;
          x(6);
          1/m*( F_x );
          1/m*( F_y );
          1/I_z*( M_z )];
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
    g = p(21);
    m = p(12);
    rho = p(23);
    T_max_front = p(14);
    T_max_rear = p(15);
    T_brake_front = p(16);
    T_brake_rear = p(17);
    GR = p(18);
    r_wheel = p(20);
    eta_motor = p(19);
    C_roll = p(22);
    C_d = p(26);

    % Side Slip Angle
    Beta = atan(l_r/(l_f+l_r)*tan(u(2)));

    % Braking or Accel Torque
    t_bool = (u(1) < 0);
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front);
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear);

    % Rolling Resistance Force
    F_roll = C_roll*m*g;
    
    % Frag Force
    F_drag = 1/2*rho*C_d*(x(4)*cos(Beta))^2;
    
    % Front and rear forces
    F_f = 2*eta_motor*T_front*GR/r_wheel*u(1);
    F_r = 2*eta_motor*T_rear*GR/r_wheel*u(1);

    % Propulsion Force
    F = F_f + F_r - F_drag - F_roll;

    % State derivative function
    dX = [x(4)*cos(x(3)+Beta);...
          x(4)*sin(x(3)+Beta);...
          x(4)*cos(Beta)/(l_f+l_r)*tan(u(2));...
          F/m];
end

% Nonlinear Inequalities
function h = ineqI(z)    
    h = [z(1)-z(3);                             % rate throttle
         z(2)-z(4)];                            % rate steering
end

function h = ineqM(z)
    [g_x, g_y, dg_x, dg_y] = splineTrack(z(6));

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;
    
    h = [dg_y*(z(7) - g_x) - dg_x*(z(8) - g_y); % e_CL 
         z(1)-z(3);                             % rate throttle
         z(2)-z(4);                             % rate steering
         z(6)-z(5)];                            % rate progress
end

function h = ineqN(z)
    [g_x, g_y, dg_x, dg_y] = splineTrack(z(2));

    temp = sqrt(dg_x^2 + dg_y^2);
    dg_x = dg_x/temp;
    dg_y = dg_y/temp;
    
    h = [dg_y*(z(3) - g_x) - dg_x*(z(4) - g_y); % e_CL 
         z(2)-z(1)];                            % rate progress
end

function [x,y,dx,dy] = splineTrack(s)
    global xSpline_FORCES;
    global ySpline_FORCES;
    global dxSpline_FORCES;
    global dySpline_FORCES;

    x = xSpline_FORCES(s);
    y = ySpline_FORCES(s);
    dx = dxSpline_FORCES(s);
    dy = dySpline_FORCES(s);
end

function [e_CL,e_L] = getErrors(x,y)
    [g_x, g_y, dg_x, dg_y] = splineTrack(z(2));
    e_CL = dg_y*(x - g_x) - dg_x*(y - g_y);
    e_L = -dg_x*(x - g_x) - dg_y*(y - g_y);
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