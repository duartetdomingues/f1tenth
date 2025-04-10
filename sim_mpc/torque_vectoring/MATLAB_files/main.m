% This script generates the necessary files to use in the MPC TV
%
% Developped by : David Gomes, April 2022,
% david.m.r.gomes@tecnico.ulisboa.pt

%% ----------   Initialization  -----------------------------------------
clear all; clc; close all
% Add yaml parser to path
addpath("yaml_parser/")

% Flags to replace files and compile
REPLACE_MATLAB = true;
REPLACE_CPP = true;
% Flags for MatLab Simulation
FULL_TEST = true;
% Flag to generate solver or just to run the simulation
GENERATE_SOLVER = true;
% Folder Name
folder_name = 'TV_solver_output';
try
    disp('Overwriting previous version of this tracks solver');
    rmdir(folder_name,'s');
catch
    disp('Creating Solver Folder');
end
mkdir(folder_name);
% Store Mex Data
global STORE_MEX_DATA
STORE_MEX_DATA = false;

%% ----------   Select config file   ------------------------------------

% [config_file,config_path] = uigetfile('../config/*.yaml', 'Select a config file');
global config_data ;
% config_data =  yaml.loadFile([config_path,config_file]);
config_data =  yaml.loadFile('../config/default.yaml');

%% ----------   Generate FORCESPRO Solver   -----------------------------
if GENERATE_SOLVER
    formulate_TV();
end

%% ----------   FULL TEST       -----------------------------------------
% Plot the whole race considering the MPC model is the car model

if FULL_TEST == true
    % Choose and read config file
%     [config_file,config_path] = uigetfile('../config/*.yaml', 'Select a config file');
%     global config_data ;
%     config_data =  yaml.loadFile([config_path,config_file]);
    
    duration = 999;
    N = config_data.mpc_params.objective.N;
    
    applied_torques = zeros(4,duration);
    
    states = zeros(2,duration);
    exit_flags = zeros(1,duration);
    solve_time = zeros(1,duration);

%     Objective function
    objective = zeros(N,duration);
    jacObjectiveInit = zeros(config_data.mpc_params.objective.z,duration);
    jacObjective = zeros(config_data.mpc_params.objective.z,N-2,duration);
    jacObjectiveN = zeros(config_data.mpc_params.objective.z-5,duration);
    sumObjective = zeros(1,duration);
    % Ineq function
    ineqInit = zeros(10,duration);
    ineq = zeros(12,N-2,duration);
    ineqN = zeros(4,duration);
    jacIneqInit = zeros(10,config_data.mpc_params.objective.z,duration);
    jacIneq = zeros(12,config_data.mpc_params.objective.z,N-2,duration);
    jacIneqN = zeros(4,config_data.mpc_params.objective.z-5,duration);
    % Dynamics function
    dynamicsInit = zeros(4,duration);
    dynamics = zeros(4,N-3,duration);
    dynamicsN = zeros(4,duration);
    jacDynamicsInit = zeros(4,config_data.mpc_params.objective.z,duration);
    jacDynamics = zeros(4,config_data.mpc_params.objective.z,N-3,duration);
    jacDynamicsN = zeros(4,config_data.mpc_params.objective.z,duration);

    allzI = zeros(config_data.mpc_params.objective.z,duration);
    allZ = zeros(config_data.mpc_params.objective.z,N-2,duration);
    allzN = zeros(config_data.mpc_params.objective.z-5,duration);
    allParams = zeros(duration,config_data.mpc_params.objective.p);
    changeParams = zeros(duration,4+4+1);

    % Set initial guess to start solver from (here, middle of upper and lower bound)
    x0iI=[0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]';
    x0iM=[0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]';
    x0iZ=[0.01,0.01,0.01,0.01,0.01]';
    x0=repmat(x0iM,N-2,1);
    
    problem.x0 = [x0iI;x0;x0iZ];

    % Define control input and angular velocity
    f = 1.0;
    t = linspace(0,5,500);
    delta = 0.66*config_data.mpc_params.ineq_limits.steering_max*sin(2*pi*f*t);
    delta = [zeros(1,500) delta];
    angular_steering = 0.66*config_data.mpc_params.ineq_limits.steering_max*2*pi*f*cos(2*pi*f*t);
    angular_steering = [zeros(1,500) angular_steering];

    % Normal Forces
    Fz_fl_lin = config_data.model_params.m*9.81*config_data.model_params.l_r/(config_data.model_params.l_r+config_data.model_params.l_f)/2;
    Fz_fr_lin = config_data.model_params.m*9.81*config_data.model_params.l_r/(config_data.model_params.l_r+config_data.model_params.l_f)/2;
    Fz_rl_lin = config_data.model_params.m*9.81*config_data.model_params.l_f/(config_data.model_params.l_r+config_data.model_params.l_f)/2;
    Fz_rr_lin = config_data.model_params.m*9.81*config_data.model_params.l_f/(config_data.model_params.l_r+config_data.model_params.l_f)/2;

    % Define throttle/requested torques and velocity
    throttle = 0.3;
    v_x = 5;
    T_front_ref = config_data.model_params.T_max_front * throttle;
    T_rear_ref = config_data.model_params.T_max_rear * throttle;

    % Get linearized parameters
    r = 0;
    Beta = 0;
    [alpha_f_lin, alpha_r_lin, Fy_fl_lin, Fy_fr_lin, Fy_rl_lin, Fy_rr_lin, C_alpha_lin_f, C_alpha_lin_r] = get_lin_params(r,Beta,delta(1),v_x,Fz_fl_lin,Fz_fr_lin,Fz_rl_lin,Fz_rr_lin);

    parameters = [config_data.model_params.T_max_front; config_data.model_params.T_max_rear; config_data.model_params.T_brake_front; config_data.model_params.T_brake_rear; throttle; config_data.model_params.K_us_ref; config_data.model_params.l_f; config_data.model_params.l_r; config_data.model_params.I_z; Fy_fr_lin; Fy_fl_lin; Fy_rr_lin; Fy_rl_lin; alpha_f_lin; alpha_r_lin; C_alpha_lin_f; C_alpha_lin_r; config_data.model_params.r_wheel; config_data.model_params.track_width; config_data.model_params.l_f+config_data.model_params.l_r; config_data.model_params.m; v_x; config_data.mpc_params.objective.w_u; config_data.mpc_params.objective.w_m; config_data.mpc_params.objective.w_r; config_data.mpc_params.objective.w_epsilon_1; config_data.mpc_params.objective.w_epsilon_2; config_data.model_params.GR; config_data.model_params.eta_motor; angular_steering(1); config_data.tyre_params.mu_x; config_data.tyre_params.mu_y; config_data.tyre_params.alpha_max; Fz_fr_lin; Fz_fl_lin; Fz_rr_lin; Fz_rl_lin; 9.81; N; T_front_ref; T_rear_ref; config_data.mpc_params.objective.k_yaw_ref];
    problem.all_parameters = repmat(parameters,N,1); % 984
    problem.solver_timeout = config_data.solver_timeout;
    problem.num_of_threads = config_data.num_of_threads;

    % Set initial conditions
    problem.xinit = [0;delta(1);0;0]; 

    % Time to solve the NLP!
    for i=1:duration
        allParams(i,:) = parameters;
        changeParams(i,:) = [parameters(10), parameters(11), parameters(12), parameters(13), parameters(14), parameters(15) ,parameters(16), parameters(17), parameters(30) ];
        % Come up with a solution and store
        [output,exit_flags(i),info] = solverTV(problem);

        % Store Data
        applied_torques(:,i) = output.zI(1:4);
        states(:,i) = output.zI(9:10);
        solve_time(i) = info.solvetime;
        allzI(:,i) = output.zI;
        resizedZ = reshape(output.z,[config_data.mpc_params.objective.z config_data.mpc_params.objective.N-2]);
        allZ(:,:,i) = resizedZ;

        if STORE_MEX_DATA
            % Store the objective funtion values and jacobians
            [objective(1,i), jacObjectiveInit(:,i)] = solverTV_objective(output.zI,parameters,1);
            for j=1:N-2
                [objective(j+1,i), jacObjective(:,j,i)] = solverTV_objective(resizedZ(:,j),parameters,j+1);
            end
            [objective(N,i), jacObjectiveN(:,i)] = solverTV_objective(output.zN,parameters,N);
            sumObjective(i) = sum(objective(:,i));

            % Store non linear ineq values and jacobians
            [ineqInit(:,i), jacIneqInit(:,:,i)] = solverTV_ineq(output.zI,parameters,1);
            for j=1:N-2
                [ineq(:,j,i), jacIneq(:,:,j,i)] = solverTV_ineq(resizedZ(:,j),parameters,j+1);
            end
            [ineqN(:,i), jacIneqN(:,:,i)] = solverTV_ineq(output.zN,parameters,N);

            % Store equality values and jacobians
            [dynamicsInit(:,i), jacDynamicsInit(:,:,i)] = solverTV_dynamics(output.zI,parameters,1);
            for j=1:N-3
                [dynamics(:,j,i), jacDynamics(:,:,j,i)] = solverTV_dynamics(resizedZ(:,j),parameters,j+1);
            end
            [dynamicsN(:,i), jacDynamicsN(:,:,i)] = solverTV_dynamics(resizedZ(:,j+1),parameters,N-1);
        end
    
        % Set initial guess
        x0iI=output.zI;
        x0iM=output.z;
        x0iZ=output.zN;
    
        problem.x0 = [x0iI;x0iM;x0iZ];

        % Update parameters
        r = resizedZ(9,1);
        Beta = resizedZ(10,1);
        [alpha_f_lin, alpha_r_lin, Fy_fl_lin, Fy_fr_lin, Fy_rl_lin, Fy_rr_lin, C_alpha_lin_f, C_alpha_lin_r] = get_lin_params(r,Beta,delta(i+1),v_x,Fz_fl_lin,Fz_fr_lin,Fz_rl_lin,Fz_rr_lin);

        parameters = [config_data.model_params.T_max_front; config_data.model_params.T_max_rear; config_data.model_params.T_brake_front; config_data.model_params.T_brake_rear; throttle; config_data.model_params.K_us_ref; config_data.model_params.l_f; config_data.model_params.l_r; config_data.model_params.I_z; Fy_fr_lin; Fy_fl_lin; Fy_rr_lin; Fy_rl_lin; alpha_f_lin; alpha_r_lin; C_alpha_lin_f; C_alpha_lin_r; config_data.model_params.r_wheel; config_data.model_params.track_width; config_data.model_params.l_f+config_data.model_params.l_r; config_data.model_params.m; v_x; config_data.mpc_params.objective.w_u; config_data.mpc_params.objective.w_m; config_data.mpc_params.objective.w_r; config_data.mpc_params.objective.w_epsilon_1; config_data.mpc_params.objective.w_epsilon_2; config_data.model_params.GR; config_data.model_params.eta_motor; angular_steering(i+1); config_data.tyre_params.mu_x; config_data.tyre_params.mu_y; config_data.tyre_params.alpha_max; Fz_fr_lin; Fz_fl_lin; Fz_rr_lin; Fz_rl_lin; 9.81; N; T_front_ref; T_rear_ref; config_data.mpc_params.objective.k_yaw_ref];
        problem.all_parameters = repmat(parameters,N,1);
    
        % xinit
        Mz_prev = config_data.model_params.eta_motor*config_data.model_params.GR/config_data.model_params.r_wheel*config_data.model_params.track_width/2*(-output.zI(1)+output.zI(2)-output.zI(3)+output.zI(4));
        steering = delta(i+1);

        problem.xinit = [Mz_prev; steering; r; Beta];
    end

    one_count = sum(exit_flags(:) == 1);
    two_count = sum(exit_flags(:) == 2);
    six_count = sum(exit_flags(:) == -6);
    seven_count = sum(exit_flags(:) == -7);
    other_count = length(exit_flags)- one_count - two_count - six_count - seven_count;
    counts = [one_count two_count six_count seven_count other_count];
    
    %% Graphs
    figure
    tiledlayout(2,4)
    nexttile
    plot(applied_torques(1,:),'-.');
    hold on
    plot(applied_torques(2,:),'-.');
    plot(applied_torques(3,:));
    plot(applied_torques(4,:));
    plot(ones(1,duration)*config_data.model_params.T_max_front,':');
    plot(ones(1,duration)*config_data.model_params.T_max_rear,':');
    plot(-ones(1,duration)*config_data.model_params.T_brake_front,':');
    plot(-ones(1,duration)*config_data.model_params.T_brake_rear,':');
    hold off
    title("Torques [N/m]")
    legend('T_{fl}','T_{fr}','T_{rl}','T_{rr}','Location','southeast');
    nexttile
    plot(rad2deg(delta(1:duration)));
    hold on
    plot(ones(1,duration)*rad2deg(config_data.mpc_params.ineq_limits.steering_min));
    plot(ones(1,duration)*rad2deg(config_data.mpc_params.ineq_limits.steering_max));
    hold off
    title('Steering [\circ]');
    nexttile
    plot(exit_flags)
    title('Exit Flags')
    nexttile
    plot(solve_time);
    title('Solve time')
    nexttile
    r_ref = config_data.mpc_params.objective.k_yaw_ref * v_x/((config_data.model_params.l_f+config_data.model_params.l_f)+config_data.model_params.K_us_ref*v_x^2)*delta(1:end-1);
    plot(rad2deg(r_ref));
    hold on
    plot(rad2deg(states(1,:)));
    plot(ones(duration-1,1)*rad2deg((config_data.tyre_params.mu_y)*9.81/v_x),':')
    plot(-ones(duration-1,1)*rad2deg((config_data.tyre_params.mu_y)*9.81/v_x),':')
    legend('Reference','Real','Location','southwest')
    title('Yaw Rate [\circ/s]')
    nexttile
    plot(rad2deg(angular_steering(1:duration)))
    title('Angular Steering Speed [\circ/s]')
    nexttile
    plot(rad2deg(states(2,:)))
    title('\beta [\circ]')
    nexttile
    plot(rad2deg(changeParams(:,5)));
    hold on 
    plot(rad2deg(changeParams(:,6)));
    plot(ones(duration-1,1)*rad2deg(config_data.tyre_params.alpha_max),':')
    plot(-ones(duration-1,1)*rad2deg(config_data.tyre_params.alpha_max),':')
    legend('\alpha_{f}','\alpha_{r}','Location','southwest')
    title('Lateral Slip Angles [\circ]')

    solve_time_str = sprintf('Mean solve time is: %f\nMax solve time is: %f\n Min solve time is: %f',mean(solve_time), max(solve_time), min(solve_time));
    disp(solve_time_str);

end

%% ----------   Change MatLab Files    ----------------------------------

if REPLACE_MATLAB
    disp("Changing headers and organizing files within MATLAB folder");
    % Replace adtool2forces includes
    find_and_replace('solverTV_adtool2forces.c','\<#include "solverTV/include/solverTV.h"\>','#include "include/solverTV.h"');
%     find_and_replace('solverTV_adtool2forces.c','\<#include "solverTV_casadi.h"\>',['#include "solverTV_casadi.h"' newline '#include "solverTV_casadi.c"']);
    
    % move files to output folder
    copyfile("solverTV_casadi.c",folder_name);
    copyfile("solverTV_casadi.h",folder_name);
    copyfile("solverTV_adtool2forces.c",folder_name);
    copyfile('solverTV/include/', [folder_name '/include/']);
    copyfile('solverTV/lib/', [folder_name '/lib/']);
    
    % move all other files created by the solver to some dump folder
    rest_folder_name = 'rest';
    mkdir(rest_folder_name);
    copyfile("*.forces",rest_folder_name);
    copyfile("*.mexa64",rest_folder_name);
    copyfile("*.o",rest_folder_name);
    try
        copyfile("*_warnings.txt",rest_folder_name);
    end
    copyfile("*_py.py",rest_folder_name);
%     copyfile("solverTV",rest_folder_name);
    movefile(rest_folder_name,folder_name);

else
    disp("NOT Changing headers and organizing files");
end
%% ----------   Change Cpp Files    -------------------------------------
if REPLACE_CPP
    disp("Replacing Cpp files");
    system('rm -r ../solverTV/');
    copyfile(folder_name,'../solverTV/');
else
    disp("NOT Replacing Cpp files");
end

%% ----------   Finalization        -------------------------------------

disp(['------------------------------------------------------',newline ,...
    'Output files will be in a folder named "', folder_name, '"'])

%CAN ADD NORMAL FORCES TO MAKE IT BETTER
function [alpha_f_lin, alpha_r_lin, Fy_fl_lin, Fy_fr_lin, Fy_rl_lin, Fy_rr_lin, C_alpha_lin_f, C_alpha_lin_r] = get_lin_params(r,Beta,delta,v_x,Fz_fl_lin,Fz_fr_lin,Fz_rl_lin,Fz_rr_lin)
    global config_data;
    a = config_data.model_params.l_f;
    b = config_data.model_params.l_r;
    D = config_data.tyre_params.D;
    C = config_data.tyre_params.C;
    B = config_data.tyre_params.B;

    alpha_f_lin = Beta - delta + a * r / v_x;
    alpha_r_lin = Beta - b * r / v_x;

    Fy_fl_lin = -D*Fz_fl_lin*sin(C*atan(B*alpha_f_lin));
    Fy_fr_lin = -D*Fz_fr_lin*sin(C*atan(B*alpha_f_lin));
    Fy_rl_lin = -D*Fz_rl_lin*sin(C*atan(B*alpha_r_lin));
    Fy_rr_lin = -D*Fz_rr_lin*sin(C*atan(B*alpha_r_lin));

    C_alpha_lin_f = -(B*C*D*(Fz_fl_lin+Fz_fr_lin)*cos(C*atan(B*alpha_f_lin)))/(B^2*alpha_f_lin^2+1);
    C_alpha_lin_r = -(B*C*D*(Fz_rl_lin+Fz_rr_lin)*cos(C*atan(B*alpha_r_lin)))/(B^2*alpha_r_lin^2+1);
end
