clear; close all
bag_path = '/home/david/bags/TV_data_real/filtered/';
bags = dir(bag_path);
method = 1; % 0 - MPC, 1 - Data Yaw Ref
h = 0.05;
ratio = 0.5;
invert = true;
add_zero_vel = true;


vel = [];
v_x = [];
v_y = [];
r = [];
Beta = [];
throttle = [];
delta = [];
angular_steering = [];

for j = 3:length(bags)
    % Get data from bag
    bag = rosbag(strcat(bag_path,bags(j).name));
    bSel = select(bag,'Topic','/control/learning_mpcc/nn_data');
    msgStructs = readMessages(bSel,'DataFormat','struct');
    for i=1:length(msgStructs)
        tmp_r(i) = double(msgStructs{i}.CarVelocity.Velocity.Theta);
        tmp_v_x(i) = double(msgStructs{i}.CarVelocity.Velocity.X);
        tmp_v_y(i) = double(msgStructs{i}.CarVelocity.Velocity.Y);
        tmp_delta(i) = double(msgStructs{i}.ControlCmd.SteeringAngle);
        tmp_throttle(i) = double(msgStructs{i}.ControlCmd.Throttle);
        if i == 1
            tmp_angular_steering(i) = tmp_delta(i)/h;
        else
            tmp_angular_steering(i) = (tmp_delta(i) - tmp_delta(i-1))/h;
        end
    end
    
    %Take lwo velocity
    tmp_vel = sqrt(tmp_v_x.^2 + tmp_v_y.^2);
    if method == 0
        idxs = tmp_vel > 2.5;
        tmp_vel = tmp_vel(idxs);
        tmp_r = tmp_r(idxs);
        tmp_v_x = tmp_v_x(idxs);
        tmp_v_y = tmp_v_y(idxs);
        tmp_delta = tmp_delta(idxs);
        tmp_throttle = tmp_throttle(idxs);
    end
    tmp_Beta = atan(tmp_v_y./tmp_v_x);

    %Append
    vel = [vel, tmp_vel];
    r = [r, tmp_r];
    v_x = [v_x, tmp_v_x];
    v_y = [v_y, tmp_v_y];
    delta = [delta, tmp_delta];
    throttle = [throttle, tmp_throttle];
    Beta = [Beta, tmp_Beta];
    angular_steering = [angular_steering, tmp_angular_steering];

end
% if invert == true
%     vel = [vel, vel];
%     r = [r, -r];
%     v_x = [v_x, v_x];
%     v_y = [v_y, -v_y];
%     delta = [delta, -delta];
%     throttle = [throttle, throttle];
%     Beta = [Beta, -Beta];
% end

addpath("yaml_parser/")
global config_data
config_data =  yaml.loadFile('../config/default.yaml');
duration = length(Beta);
N = config_data.mpc_params.objective.N;

if method == 0
    % Normal Forces
    Fz_fl_lin = config_data.model_params.m*9.81*config_data.model_params.l_r/(config_data.model_params.l_r+config_data.model_params.l_f)/2;
    Fz_fr_lin = config_data.model_params.m*9.81*config_data.model_params.l_r/(config_data.model_params.l_r+config_data.model_params.l_f)/2;
    Fz_rl_lin = config_data.model_params.m*9.81*config_data.model_params.l_f/(config_data.model_params.l_r+config_data.model_params.l_f)/2;
    Fz_rr_lin = config_data.model_params.m*9.81*config_data.model_params.l_f/(config_data.model_params.l_r+config_data.model_params.l_f)/2;
    
    [alpha_f_lin, alpha_r_lin, Fy_fl_lin, Fy_fr_lin, Fy_rl_lin, Fy_rr_lin, C_alpha_lin_f, C_alpha_lin_r] = get_lin_params(r,Beta,delta,v_x,Fz_fl_lin,Fz_fr_lin,Fz_rl_lin,Fz_rr_lin);
    
    % Set initial guess to start solver from (here, middle of upper and lower bound)
    x0iI=[0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]';
    x0iM=[0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]';
    x0iZ=[0.01,0.01,0.01,0.01,0.01]';
    x0=repmat(x0iM,N-2,1);
    
    problem.x0 = [x0iI;x0;x0iZ];
    problem.solver_timeout = config_data.solver_timeout;
    problem.num_of_threads = config_data.num_of_threads;
    for i = 1:duration
        if throttle >= 0
            T_front_ref = throttle(i)*config_data.model_params.T_max_front;
            T_rear_ref = throttle(i)*config_data.model_params.T_rear_front;
        else
            T_front_ref = throttle(i)*config_data.model_params.T_brake_front;
            T_rear_ref = throttle(i)*config_data.model_params.T_brake_front;
        end
        parameters = [config_data.model_params.T_max_front; config_data.model_params.T_max_rear; config_data.model_params.T_brake_front; config_data.model_params.T_brake_rear; throttle(i); config_data.model_params.K_us_ref; config_data.model_params.l_f; config_data.model_params.l_r; config_data.model_params.I_z; Fy_fr_lin(i); Fy_fl_lin(i); Fy_rr_lin(i); Fy_rl_lin(i); alpha_f_lin(i); alpha_r_lin(i); C_alpha_lin_f(i); C_alpha_lin_r(i); config_data.model_params.r_wheel; config_data.model_params.track_width; config_data.model_params.l_f+config_data.model_params.l_r; config_data.model_params.m; v_x(i); config_data.mpc_params.objective.w_u; config_data.mpc_params.objective.w_m; config_data.mpc_params.objective.w_r; config_data.mpc_params.objective.w_epsilon_1; config_data.mpc_params.objective.w_epsilon_2; config_data.model_params.GR; config_data.model_params.eta_motor; angular_steering(i); config_data.tyre_params.mu_x; config_data.tyre_params.mu_y; config_data.tyre_params.alpha_max; Fz_fr_lin; Fz_fl_lin; Fz_rr_lin; Fz_rl_lin; 9.81; N; T_front_ref; T_rear_ref; config_data.mpc_params.objective.k_yaw_ref];
        problem.all_parameters = repmat(parameters,N,1);
        problem.xinit = [0; delta(i); r(i); Beta(i)];
    
        [output,exit_flags(i),info] = solverTV(problem);
        if exit_flags(i) == 1
            applied_torques(:,i) = output.zI(1:4);
            T_front_ref_saved(i) = T_front_ref;
            T_rear_ref_saved(i) = T_rear_ref;
        end
    end

    L = config_data.model_params.l_f + config_data.model_params.l_r;
    K_us = config_data.model_params.K_us_ref;
    desired_r = v_x./(L+K_us.*v_x.^2).*delta;
    
    figure
    tiledlayout(2,2)
    nexttile
    hold on
    plot(applied_torques(1,:),'-.');
    plot(applied_torques(2,:),'-.');
    plot(applied_torques(3,:));
    plot(applied_torques(4,:));
    plot(T_front_ref_saved,':');
    plot(T_rear_ref_saved,':');
    legend('FL','FR','RL','RR','F\_ref','R\_ref',Location='southeast');
    nexttile
    plot(rad2deg(delta));
    title('Steering [\circ]');
    nexttile
    plot(r)
    hold on
    plot(desired_r)
    legend('Actual','Reference');
    nexttile
    plot(vel)
    title('Velocity[m/s]')
    
    diff_front = (applied_torques(1,:)- applied_torques(2,:))./2;
    diff_rear = (applied_torques(3,:)- applied_torques(4,:))./2;
elseif method == 1
    r_next = [r(2:end) r(end)];
    d_r = (r_next-r)./h;
    L = config_data.model_params.l_f + config_data.model_params.l_r;
    K_us = config_data.model_params.K_us_ref;
    desired_r = v_x./(L+K_us.*v_x.^2).*delta;
    diff_r = d_r*h - (desired_r - r_next);
    Mz = diff_r*config_data.model_params.I_z;
    diff_F = 2*Mz/config_data.model_params.track_width;
    diff_T = diff_F*config_data.model_params.r_wheel/config_data.model_params.eta_motor/config_data.model_params.GR;
    diff_rear = diff_T*1/(1+ratio);
    diff_front = diff_T-diff_rear;
    if invert == true
        v_x = [v_x, v_x];
        vel = [vel, vel];
        delta = [delta, -delta];
        diff_T = [diff_T, -diff_T];
    end
    if add_zero_vel == true
        v_x = [v_x, zeros(1,781)];
        vel = [vel, zeros(1,781)];
        add_delta = -0.39:0.001:0.39;
        delta = [delta, add_delta];
        diff_T = [diff_T, zeros(1,781)];
    end
end

function [alpha_f_lin, alpha_r_lin, Fy_fl_lin, Fy_fr_lin, Fy_rl_lin, Fy_rr_lin, C_alpha_lin_f, C_alpha_lin_r] = get_lin_params(r,Beta,delta,v_x,Fz_fl_lin,Fz_fr_lin,Fz_rl_lin,Fz_rr_lin)
    global config_data;
    a = config_data.model_params.l_f;
    b = config_data.model_params.l_r;
    D = config_data.tyre_params.D;
    C = config_data.tyre_params.C;
    B = config_data.tyre_params.B;

    alpha_f_lin = Beta - delta + a .* r ./ v_x;
    alpha_r_lin = Beta - b .* r ./ v_x;

    Fy_fl_lin = -D.*Fz_fl_lin.*sin(C.*atan(B.*alpha_f_lin));
    Fy_fr_lin = -D.*Fz_fr_lin.*sin(C.*atan(B.*alpha_f_lin));
    Fy_rl_lin = -D.*Fz_rl_lin.*sin(C.*atan(B.*alpha_r_lin));
    Fy_rr_lin = -D.*Fz_rr_lin.*sin(C.*atan(B.*alpha_r_lin));

    C_alpha_lin_f = -(B.*C.*D.*(Fz_fl_lin+Fz_fr_lin).*cos(C.*atan(B.*alpha_f_lin)))./(B.^2.*alpha_f_lin.^2+1);
    C_alpha_lin_r = -(B.*C.*D.*(Fz_rl_lin+Fz_rr_lin).*cos(C.*atan(B.*alpha_r_lin)))./(B.^2.*alpha_r_lin.^2+1);
end