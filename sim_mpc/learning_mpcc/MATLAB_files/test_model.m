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
addpath("yaml_parser/")
config_data =  yaml.loadFile('../config/default.yaml');
global integrator_stepsize
global diff_Coeffs
integrator_stepsize = 0.05;
diff_Coeffs = [-0.6574, -1.83, 0.0221];
parameters = [config_data.mpc_params.objective.alpha_CL config_data.mpc_params.objective.alpha_L config_data.mpc_params.objective.q_r config_data.mpc_params.objective.e_CL_exp config_data.mpc_params.objective.d_max config_data.mpc_params.objective.beta_steer config_data.mpc_params.objective.lambda config_data.mpc_params.objective.v_max config_data.mpc_params.objective.q_v_max config_data.model_params.l_f config_data.model_params.l_r config_data.model_params.m config_data.model_params.I_z config_data.model_params.T_max_front config_data.model_params.T_max_rear config_data.model_params.T_brake_front config_data.model_params.T_brake_rear config_data.model_params.GR config_data.model_params.eta_motor config_data.model_params.r_wheel config_data.model_params.g config_data.model_params.C_roll config_data.model_params.rho config_data.model_params.lambda_blend_min config_data.model_params.lambda_blend_max config_data.model_params.C_d config_data.model_params.C_l config_data.tyre_params.B config_data.tyre_params.C config_data.tyre_params.D config_data.model_params.downforce_front config_data.model_params.downforce_rear config_data.mpc_params.objective.beta_throttle config_data.model_params.h_cog config_data.tyre_params.mu_x config_data.tyre_params.mu_y config_data.mpc_params.ineq_limits.e_CL_max config_data.mpc_params.ineq_limits.delta_s_max config_data.mpc_params.ineq_limits.throttle_max config_data.mpc_params.ineq_limits.steering_max config_data.mpc_params.ineq_limits.delta_throttle_max config_data.mpc_params.ineq_limits.delta_steering_max config_data.model_params.width config_data.model_params.length config_data.mpc_params.ineq_limits.safety_margin config_data.mpc_params.objective.beta_epsilon config_data.mpc_params.objective.N config_data.controller_freq config_data.NN_config.NN config_data.mpc_params.objective.p config_data.mpc_params.objective.beta_psi config_data.model_params.track_width config_data.model_params.diff_gain 0.8 0.1 0];

% x = [0, 0, 0, 1, 0.1, 0.2];
% u = [0.1, 0.1];
x = [0, 0, 0, 1, 0.1, 0.1];
u = [0.1, 0.2];

dyn = dynamic_bicycle_model(x,u,parameters)
planar = planar_model(x,u,parameters)

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

%     disp(x)
%     disp(u)
%     disp(p(end-2:end))

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
    T_front = ((~t_bool)*T_max_front + t_bool*(T_brake_front)) * u(1);
    T_rear = ((~t_bool)*T_max_rear + t_bool*(T_brake_rear)) * u(1);

    % Longitudinal Load Transfer
    a_x = get_ax(x(4),p(end-2),x(5),x(6));
    a_y = get_ay(x(5),p(end-1),x(4),x(6));
    delta_Fz_x = h_cog/(l_r+l_f)*m*a_x;
    delta_Fz_y = h_cog/track_width*m*a_y;

    % Differential
    [diff_front, diff_rear] = differential(x(4),u(2),a_x,diff_gain);

    % Saturate
    T_fl = ForcesMax(ForcesMin(T_front + diff_front/2,T_max_front),-T_brake_front);
    T_fr = ForcesMax(ForcesMin(T_front - diff_front/2,T_max_front),-T_brake_front);
    T_rl = ForcesMax(ForcesMin(T_rear + diff_rear/2,T_max_rear),-T_brake_rear);
    T_rr = ForcesMax(ForcesMin(T_rear - diff_rear/2,T_max_rear),-T_brake_rear);
    
    % Get longitudinal Forces
    Fx_fl = eta_motor*T_fl*GR/r_wheel;
    Fx_fr = eta_motor*T_fr*GR/r_wheel;
    Fx_rl = eta_motor*T_rl*GR/r_wheel;
    Fx_rr = eta_motor*T_rr*GR/r_wheel;

    % Fet differential Moment
    diff_Mz = track_width/2*((Fx_fr-Fx_fl)*cos(u(2))+(Fx_rr-Fx_rl));

    % Static weight distribution
    Ff_z_static = m * (l_r/(l_f + l_r)) * g;
    Fr_z_static = m * (l_f/(l_f + l_r)) * g;

    % Downforce Distribution
    F_downforce = 0.5 * rho * C_l * x(4)^2;

    Ff_downforce = F_downforce * downforce_front;
    Fr_downforce = F_downforce * downforce_rear;

    % Slip Angles
    alpha_fl = u(2) - atan((x(5) + x(6)*l_f) / (x(4) - x(6)*track_width/2));
    alpha_fr = u(2) - atan((x(5) + x(6)*l_f) / (x(4) + x(6)*track_width/2));
    alpha_rl = -atan((x(5) - x(6)*l_r) / (x(4) - x(6)*track_width/2));
    alpha_rr = -atan((x(5) - x(6)*l_r) / (x(4) + x(6)*track_width/2));

    % Static + Downforce + Longitudinal Load Transfer
    Fz_fl = Ff_z_static/2 + Ff_downforce/2 - delta_Fz_x - delta_Fz_y;
    Fz_fr = Ff_z_static/2 + Ff_downforce/2 - delta_Fz_x + delta_Fz_y;
    Fz_rl = Fr_z_static/2 + Fr_downforce/2 + delta_Fz_x - delta_Fz_y;
    Fz_rr = Fr_z_static/2 + Fr_downforce/2 + delta_Fz_x + delta_Fz_y;

    % Lateral Forces
    Fy_fl = Fz_fl*D*sin(C*atan(B*alpha_fl));
    Fy_fr = Fz_fr*D*sin(C*atan(B*alpha_fr));
    Fy_rl = Fz_rl*D*sin(C*atan(B*alpha_rl));
    Fy_rr = Fz_rr*D*sin(C*atan(B*alpha_rr));
    Ff_tire = Fy_fl + Fy_fr;
    Fr_tire = Fy_rl + Fy_rr;

    % Longitudinal Forces
    F_f = Fx_fl + Fx_fr;
    F_r = Fx_rl + Fx_rr;
    
    % Longitudinal and Lateral Force and Yaw Moment
    F_x = F_f*cos(u(2)) - Ff_tire*sin(u(2)) + F_r - F_roll - F_drag;
    F_y = F_f*sin(u(2)) + Ff_tire*cos(u(2)) + Fr_tire;
    M_z = (F_f*sin(u(2)) + Ff_tire*cos(u(2)))*l_f - Fr_tire*l_r + track_width/2*((Fy_fl-Fy_fr)*sin(u(2)) + (Fx_fr-Fx_fl)*cos(u(2)) + (Fx_rr-Fx_rl));

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

function dX = planar_model(x,u,p)
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

%     disp(x)
%     disp(u)
%     disp(p(end-2:end))

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
    T_front = ((~t_bool)*T_max_front + t_bool*(T_brake_front)) * u(1);
    T_rear = ((~t_bool)*T_max_rear + t_bool*(T_brake_rear)) * u(1);

    % Longitudinal Load Transfer
    a_x = get_ax(x(4),p(end-2),x(5),x(6));
    a_y = get_ay(x(5),p(end-1),x(4),x(6));
    delta_Fz_x = h_cog/(l_r+l_f)*m*a_x/2;
    % delta_Fz_y = h_cog/track_width*m*a_y;
    delta_Fz_y_f = l_r/(l_r+l_f)/track_width*m*a_y*0.0368;
    delta_Fz_y_r = l_f/(l_r+l_f)/track_width*m*a_y*0.06;

    % Differential
    [diff_front, diff_rear] = differential(x(4),u(2),a_x,diff_gain);

    % Saturate
    T_fl = ForcesMax(ForcesMin(T_front + diff_front/2,T_max_front),-T_brake_front);
    T_fr = ForcesMax(ForcesMin(T_front - diff_front/2,T_max_front),-T_brake_front);
    T_rl = ForcesMax(ForcesMin(T_rear + diff_rear/2,T_max_rear),-T_brake_rear);
    T_rr = ForcesMax(ForcesMin(T_rear - diff_rear/2,T_max_rear),-T_brake_rear);
    
    % Get longitudinal Forces
    Fx_fl = eta_motor*T_fl*GR/r_wheel;
    Fx_fr = eta_motor*T_fr*GR/r_wheel;
    Fx_rl = eta_motor*T_rl*GR/r_wheel;
    Fx_rr = eta_motor*T_rr*GR/r_wheel;

    % Static weight distribution
    Ff_z_static = m * (l_r/(l_f + l_r)) * g;
    Fr_z_static = m * (l_f/(l_f + l_r)) * g;

    % Downforce Distribution
    F_downforce = 0.5 * rho * C_l * x(4)^2;

    Ff_downforce = F_downforce * downforce_front;
    Fr_downforce = F_downforce * downforce_rear;

    % Slip Angles
    alpha_fl = u(2) - atan((x(5) + x(6)*l_f) / (x(4) - x(6)*track_width/2));
    alpha_fr = u(2) - atan((x(5) + x(6)*l_f) / (x(4) + x(6)*track_width/2));
    alpha_rl = -atan((x(5) - x(6)*l_r) / (x(4) - x(6)*track_width/2));
    alpha_rr = -atan((x(5) - x(6)*l_r) / (x(4) + x(6)*track_width/2));

    % Static + Downforce + Longitudinal Load Transfer
    Fz_fl = Ff_z_static/2 + Ff_downforce/2 - delta_Fz_x - delta_Fz_y_f;
    Fz_fr = Ff_z_static/2 + Ff_downforce/2 - delta_Fz_x + delta_Fz_y_f;
    Fz_rl = Fr_z_static/2 + Fr_downforce/2 + delta_Fz_x - delta_Fz_y_r;
    Fz_rr = Fr_z_static/2 + Fr_downforce/2 + delta_Fz_x + delta_Fz_y_r;

    % Lateral Forces
    Fy_fl = Fz_fl*D*sin(C*atan(B*alpha_fl));
    Fy_fr = Fz_fr*D*sin(C*atan(B*alpha_fr));
    Fy_rl = Fz_rl*D*sin(C*atan(B*alpha_rl));
    Fy_rr = Fz_rr*D*sin(C*atan(B*alpha_rr));
    Ff_tire = Fy_fl + Fy_fr;
    Fr_tire = Fy_rl + Fy_rr;

    % Longitudinal Forces
    F_f = Fx_fl + Fx_fr;
    F_r = Fx_rl + Fx_rr;
    
    % Longitudinal and Lateral Force and Yaw Moment
    F_x = F_f*cos(u(2)) - Ff_tire*sin(u(2)) + F_r - F_roll - F_drag;
    F_y = F_f*sin(u(2)) + Ff_tire*cos(u(2)) + Fr_tire;
    M_z = (F_f*sin(u(2)) + Ff_tire*cos(u(2)))*l_f - Fr_tire*l_r + track_width/2*((Fy_fl-Fy_fr)*sin(u(2)) + (Fx_fr-Fx_fl)*cos(u(2)) + (Fx_rr-Fx_rl));

    % State derivative function
    cos_Phi = cos(x(3));
    sin_Phi = sin(x(3));
    dX = [x(4)*cos_Phi - x(5)*sin_Phi;
          x(4)*sin_Phi + x(5)*cos_Phi;
          x(6);
          F_x/m + x(5)*x(6);
          F_y/m - x(4)*x(6);
          M_z/I_z];
    fprintf("Fx: %f, Fy: %f, Mz: %f, F_f: %f, F_r : %f, Fy_fl : %f, Fy_fr: %f, Fy_rl : %f, Fy_rr: %f, alpha_rr: %f, alpha_fl: %f, alpha_fr: %f, alpha_rl: %f",F_x,F_y,M_z,F_f,F_r,Fy_fl,Fy_fr,Fy_rl,Fy_rr, alpha_fl, alpha_fr, alpha_rl, alpha_rr)
end

function a_x = get_ax(vx,vx_prev, vy, r)
    global integrator_stepsize
    a_x = (vx-vx_prev)/integrator_stepsize - vy*r;
end

function a_y = get_ay(vy,vy_prev,vx,r)
    global integrator_stepsize
    a_y = (vy-vy_prev)/integrator_stepsize + vx*r;
end

function [diff_front, diff_rear] = differential(v_x,delta,a_x,diff_gain)
    global diff_Coeffs;
    ratio = ForcesMax(ForcesMin(a_x/9.81,1),-1)*0.16;

    diff_T = diff_gain*(diff_Coeffs(1)*delta + diff_Coeffs(2)*v_x*delta + diff_Coeffs(3)*delta^3);
    diff_front = diff_T*(0.4-ratio);
    diff_rear = diff_T*(0.6+ratio);
end