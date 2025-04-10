addpath("yaml_parser/")

x = [0;0;0;1;1;1];
u = [1;0.39];


config_data =  yaml.loadFile('../config/default.yaml');


    p = [config_data.mpc_params.objective.alpha_CL config_data.mpc_params.objective.alpha_L config_data.mpc_params.objective.q_r config_data.mpc_params.objective.e_CL_exp config_data.mpc_params.objective.d_max config_data.mpc_params.objective.beta_steer config_data.mpc_params.objective.lambda config_data.mpc_params.objective.v_max config_data.mpc_params.objective.q_v_max config_data.model_params.l_f config_data.model_params.l_r config_data.model_params.m config_data.model_params.I_z config_data.model_params.T_max_front config_data.model_params.T_max_rear config_data.model_params.T_brake_front config_data.model_params.T_brake_rear config_data.model_params.GR config_data.model_params.eta_motor config_data.model_params.r_wheel config_data.model_params.g config_data.model_params.C_roll config_data.model_params.rho config_data.model_params.lambda_blend_min config_data.model_params.lambda_blend_max config_data.model_params.C_d config_data.model_params.C_l config_data.tyre_params.B config_data.tyre_params.C config_data.tyre_params.D config_data.model_params.downforce_front config_data.model_params.downforce_rear config_data.mpc_params.objective.beta_throttle config_data.model_params.h_cog config_data.tyre_params.mu_x config_data.tyre_params.mu_y config_data.mpc_params.ineq_limits.e_CL_max config_data.mpc_params.ineq_limits.delta_s_max config_data.mpc_params.ineq_limits.throttle_max config_data.mpc_params.ineq_limits.steering_max config_data.mpc_params.ineq_limits.delta_throttle_max config_data.mpc_params.ineq_limits.delta_steering_max config_data.model_params.width config_data.model_params.length config_data.mpc_params.ineq_limits.safety_margin config_data.mpc_params.objective.beta_epsilon config_data.mpc_params.objective.N config_data.controller_freq config_data.NN_config.NN config_data.mpc_params.objective.p config_data.mpc_params.objective.beta_psi config_data.model_params.track_width config_data.model_params.diff_gain]';

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
    
    % Braking or Accel Torque
    t_bool = (u(1) < 0);
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front);
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear);

    % Differential
    F_fl = eta_motor*T_front*GR/r_wheel*u(1);
    F_fr = eta_motor*T_front*GR/r_wheel*u(1);
    F_rl = eta_motor*T_rear*GR/r_wheel*u(1);
    F_rr = eta_motor*T_rear*GR/r_wheel*u(1);

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

    % Slip Angles vx+1.5./(1+vx))
    alpha_f = u(2) - atan2(x(5) + x(6)*l_f, x(4));
    alpha_r = atan2((x(5) - x(6)*l_r), x(4));

    % Longitudinal Load Transfer
%     F_f_ax = 2*eta_motor*T_front*GR/r_wheel*u(3);
%     F_r_ax = 2*eta_motor*T_rear*GR/r_wheel*u(3);
%     Ff_tire_load = (Ff_z_static+Ff_downforce)*D*sin(C*atan(B*alpha_f));
%     a_x = (F_f_ax*cos(u(4)) - Ff_tire_load*sin(u(4)) + F_r_ax - F_roll - F_drag)/m + x(5)*x(6);
%     delta_Fz = h_cog/(l_r+l_f)*m*a_x;
%     delta_Fz = 0;

    % Static + Downforce + Longitudinal Load Transfer
    Ff_z = Ff_z_static + Ff_downforce;
    Fr_z = Fr_z_static + Fr_downforce;

    % Lateral Forces
    Ff_tire = Ff_z*D*sin(C*atan(B*alpha_f));
    Fr_tire = Fr_z*D*sin(C*atan(B*alpha_r));
   
    % Longitudinal Forces
    F_f = F_fl + F_fr;
    F_r = F_rl + F_rr;
    
    % Longitudinal and Lateral Force and Yaw Moment
    F_x = F_f*cos(u(2)) - Ff_tire*sin(u(2)) + F_r - F_roll - F_drag;
    F_y = F_f*sin(u(2)) + Ff_tire*cos(u(2)) - Fr_tire;
    M_z = (F_f*sin(u(2)) + Ff_tire*cos(u(2)))*l_f + Fr_tire*l_r;

    % NN correction
%     NN_input = [u(3) u(4) p(end-2) p(end-1) p(end); u(1) u(2) x(4) x(5) x(6)];
%     NN_correction = NN_flag * NNmodel(NN_input,p(1:end-3));
    
    % State derivative function
    cos_Phi = cos(x(3));
    sin_Phi = sin(x(3));
    dX = [x(4)*cos_Phi - x(5)*sin_Phi;
          x(4)*sin_Phi + x(5)*cos_Phi;
          x(6);
          F_x/m + x(5)*x(6);
          F_y/m - x(4)*x(6);
          M_z/I_z]

function [diff_front, diff_rear] = differential(delta,diff_gain)
    global diffFront_Coeffs;
    global diffRear_Coeffs;

    lut_front = diffFront_Coeffs(1)*delta^3 + diffFront_Coeffs(2)*delta^2 + diffFront_Coeffs(3)*delta + diffFront_Coeffs(4);
    lut_rear = diffRear_Coeffs(1)*delta + diffRear_Coeffs(2);

    diff_front = diff_gain*ForcesMax(ForcesMin(2*lut_front,0.0095*1000),-0.0095*1000);
    diff_rear = diff_gain*ForcesMax(ForcesMin(2*lut_rear,0.0207*1000),-0.0207*1000);
end