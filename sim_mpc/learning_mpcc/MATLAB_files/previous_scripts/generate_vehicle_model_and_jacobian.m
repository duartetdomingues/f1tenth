clear all; clc; close all
addpath("yaml_parser/")
config_data =  yaml.loadFile('../config/default.yaml');
p = [config_data.mpc_params.objective.alpha_CL config_data.mpc_params.objective.alpha_L config_data.mpc_params.objective.q_r config_data.mpc_params.objective.e_CL_exp config_data.mpc_params.objective.d_max config_data.mpc_params.objective.beta_steer config_data.mpc_params.objective.lambda config_data.mpc_params.objective.v_max config_data.mpc_params.objective.q_v_max config_data.model_params.l_f config_data.model_params.l_r config_data.model_params.m config_data.model_params.I_z config_data.model_params.T_max_front config_data.model_params.T_max_rear config_data.model_params.T_brake_front config_data.model_params.T_brake_rear config_data.model_params.GR config_data.model_params.eta_motor config_data.model_params.r_wheel config_data.model_params.g config_data.model_params.C_roll config_data.model_params.rho config_data.model_params.lambda_blend_min config_data.model_params.lambda_blend_max config_data.model_params.C_d config_data.model_params.C_l config_data.tyre_params.B config_data.tyre_params.C config_data.tyre_params.D config_data.model_params.downforce_front config_data.model_params.downforce_rear config_data.mpc_params.objective.beta_throttle config_data.model_params.h_cog config_data.tyre_params.mu_x config_data.tyre_params.mu_y config_data.mpc_params.ineq_limits.e_CL_max config_data.mpc_params.ineq_limits.delta_s_max config_data.mpc_params.ineq_limits.throttle_max config_data.mpc_params.ineq_limits.steering_max config_data.mpc_params.ineq_limits.delta_throttle_max config_data.mpc_params.ineq_limits.delta_steering_max config_data.mpc_params.objective.beta_epsilon config_data.model_params.width config_data.model_params.length config_data.mpc_params.ineq_limits.safety_margin]';
global integrator_stepsize
integrator_stepsize = 0.05;
SPLINE_POINTS = 55;
EXTENDED_POINTS = 5; % Zero is currently not supported
global matSpline

[matSpline, track_length, folder_name] = spline_creation(SPLINE_POINTS,EXTENDED_POINTS);

% Normal Case
syms z [11 1]
x = z(6:11);
u = z(1:4);
f = [z(1); z(2); progress_dynamics(z(5),z(8),z(9),z(10)); vehicle_dynamics(x,u,p);];
J = jacobian(f, z);
% generate C code for the jacobian computation
args = struct;
args.x = zeros(11,1);
cfg = coder.config('lib');
codegen -config cfg -args {coder.typeof(args.x,[11 1])} f -o dynamics_eq_matlab
codegen -config cfg -args {coder.typeof(args.x,[11 1])} J -o dynamics_jacobian_matlab

% Last step
f_last = [progress_dynamics(z(5),z(8),z(9),z(10)); vehicle_dynamics(x,u,p);];
J_last = jacobian(f_last, z);
codegen -config cfg -args {coder.typeof(args.x,[11 1])} f_last -o dynamics_eq_matlab
codegen -config cfg -args {coder.typeof(args.x,[11 1])} J_last -o dynamics_jacobian_matlab

% Progress Dynamics
function progress = progress_dynamics(s,yaw,vx,vy)
    global integrator_stepsize;

    [dg_x, dg_y] = splineTrack(s);
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
    dX_dyn = dynamic_bicycle_model(x,u,p);
    dyn_bike = euler_step(x,dX_dyn,integrator_stepsize);

    %Discritized Kinematic Vehicle Model
    dX_kin = kinematic_bicycle_model(x,u,p);
    kin_bike = euler_step(x,dX_kin,integrator_stepsize);
    kin_bike(6) = kin_bike(4)/(l_r+l_f)*tan(u(2));
    
    % blend the two models
    velocity = sqrt(x(4)^2+x(5)^2);
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
    h_cog = p(34);

    % Braking or Accel Torque
    t_bool = (u(1) < 0);
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front);
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear);

    % Rolling Resistance
    F_roll = C_roll*m*g;
    
    % Drag Force
    F_drag = 1/2*rho*C_d*x(4)^2;

    % Static weight distribution
    Ff_z_static = m * (l_r/(l_f + l_r)) * g;
    Fr_z_static = m * (l_f/(l_f + l_r)) * g;

    % Downforce Distribution
    F_downforce = 0.5 * rho * C_l * x(4)^2;

    Ff_downforce = F_downforce * downforce_front;
    Fr_downforce = F_downforce * downforce_rear;

    % Slip Angles
    alpha_f = u(2) - atan2(x(5) + x(6)*l_f, x(4));
    alpha_r = atan2((x(5) - x(6)*l_r), x(4));

    % Longitudinal Load Transfer
    F_f_ax = 2*eta_motor*T_front*GR/r_wheel*u(3);
    F_r_ax = 2*eta_motor*T_rear*GR/r_wheel*u(3);
    Ff_tire_load = (Ff_z_static+Ff_downforce)*D*sin(C*atan(B*alpha_f));
    a_x = (F_f_ax*cos(u(4)) - Ff_tire_load*sin(u(4)) + F_r_ax - F_roll - F_drag)/m;
    delta_Fz = h_cog/(l_r+l_f)*m*a_x;
%     delta_Fz = 0;

    % Static + Downforce + Longitudinal Load Transfer
    Ff_z = Ff_z_static + Ff_downforce - delta_Fz;
    Fr_z = Fr_z_static + Fr_downforce + delta_Fz;

    % Lateral Forces
    Ff_tire = Ff_z*D*sin(C*atan(B*alpha_f));
    Fr_tire = Fr_z*D*sin(C*atan(B*alpha_r));
   
    % Torque Requests
    F_f = 2*eta_motor*T_front*GR/r_wheel*u(1);
    F_r = 2*eta_motor*T_rear*GR/r_wheel*u(1);
    
    % Longitudinal and Lateral Force and Yaw Moment
    F_x = F_f*cos(u(2)) - Ff_tire*sin(u(2)) + F_r - F_roll - F_drag;
    F_y = F_f*sin(u(2)) + Ff_tire*cos(u(2)) - Fr_tire;
    M_z = (F_f*sin(u(2)) + Ff_tire*cos(u(2)))*l_f + Fr_tire*l_r;
    
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

    % Fx and Fy
    F_x = F_f * cos(u(2)) + F_r - F_drag - F_roll;
    F_y = F_f * sin(u(2));

    % State derivative function
    dX = [x(4)*cos(x(3)+Beta);...
          x(4)*sin(x(3)+Beta);...
          x(4)*cos(Beta)/(l_f+l_r)*tan(u(2));...
          F_x/m; ...
          F_y/m; ...
          0];
end

function y = euler_step(x,dX,h)
    y = x + dX*h;
end

function [dx,dy] = splineTrack(s)
    global matSpline;
    dx_sym = sym(matSpline.dxSpline.coefs);
    dy_sym = sym(matSpline.dySpline.coefs);

    dx = ppval(dx_sym,s);
    dy = ppval(dy_sym,s);
end