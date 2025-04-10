% This script generates the necessary files to use in the MPC controller
%
% Developped by : David Gomes, November 2022,
% david.m.r.gomes@tecnico.ulisboa.pt
% Kudos to Grabriel Costa, gafc@live.com.pt, for the initial ideia

%% ----------   Initialization  -----------------------------------------
clear all; clc; close all
% Add yaml parser to path
addpath("yaml_parser/")

% NN
LAYERS = 1; % Number of Decode/Encode Layers

% Flag to generate solver or just to run the simulation
GENERATE_SOLVER = true;

% Flag to store extra debug information
STORE_MEX_DATA = false;

% Flags to replace files and compile
REPLACE_MATLAB = true;
REPLACE_CPP = true;

% Flags for MatLab Simulation
UNIT_TEST = false;
FULL_TEST = true;

% Folder Name
folder_name = 'output_agnostic_LMPC';
try
    rmdir(folder_name,'s');
    disp('Overwriting previous version of this tracks solver');
catch
    disp('Creating Solver Folder');
end
mkdir(folder_name);
mkdir(strcat(folder_name,'/include'))
mkdir(strcat(folder_name,'/lib'))

%% ----------   Select config file   ------------------------------------

% [config_file,config_path] = uigetfile('../config/*.yaml', 'Select a config file');
% config_data =  yaml.loadFile([config_path,config_file]);
global config_data
config_data =  yaml.loadFile('../config/default.yaml');

%% ----------   Generate Diff Coeffs ------------------------------------
diffCoeffs = [-0.6574, -1.83, 0.0221];

% p00 =   8.746e-16; constant
% p10 =   4.164e-16; vx
% p01 =     -0.6574; delta
% p11 =       -1.83; vx*delta
% p02 =  -5.454e-16; delta 2
% p12 =  -2.512e-16; vx*delta2
% p03 =      0.0221; delta 3

%% ----------   Generate NN Model  --------------------------------------

modelName = config_data.NN_config.model_name;
NNparams = get_NN_params_torch(modelName,LAYERS);

%% ----------   Generate FORCESPRO Solver   -----------------------------
if GENERATE_SOLVER
%     formulateMPCC_agnostic_noNN_soft_friction(STORE_MEX_DATA,diffCoeffs);
%     formulateMPCC_agnostic_NN_soft_friction(STORE_MEX_DATA,diffCoeffs);
%     formulateMPCC_agnostic_NN_soft_friction_ax_ay(STORE_MEX_DATA,diffCoeffs);
    formulateMPCC_agnostic_NN_soft_friction_noNN_ax_ay_planar(STORE_MEX_DATA,diffCoeffs);
end

%% ----------   UNIT TESTING   ------------------------------------------
if UNIT_TEST == true

    N = config_data.mpc_params.objective.N;

    % Set initial guess to start solver from (here, middle of upper and lower bound)

    % Set initial guess to start solver from (here, middle of upper and lower bound)
    x0i=[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]';
    x0iN_1 = [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]';
    x0iN=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]';
    x0=repmat(x0i,N-2,1);
    
    problem.x0 = [x0;x0iN_1;x0iN];

    % Other parameters
    n_points = config_data.mpc_params.objective.spline;
    x = linspace(0,10,n_points);
    r = 10;
    y = sqrt(r.^2-x.^2);
    y = 10*ones(1,n_points);

    
    % Set initial condition
    problem.xinit = [0,0,0,0,0,0,0,0,0,r,0,0,0,0]';

    s = zeros(n_points,1);
    for i=1:n_points-1
        s(i+1) = s(i) + sqrt((y(i+1)-y(i))^2 + (x(i+1)-x(i))^2);
    end

    s = linspace(0,s(n_points),n_points);
    
    pp_x = spline(s,x);
    pp_y = spline(s,y);
    
    pp_x_coefs = flip(pp_x.coefs,2);
    pp_y_coefs = flip(pp_y.coefs,2);
    
    for i = 1:n_points-1
        for j = 1:4
            coefs((i-1)*4+j) =  pp_x_coefs(i,j);
        end
    end
    for i = 1:n_points-1
        for j = 1:4
            coefs((n_points-1)*4+(i-1)*4+j) =  pp_y_coefs(i,j);
        end
    end


    parameters = [config_data.mpc_params.objective.alpha_CL config_data.mpc_params.objective.alpha_L config_data.mpc_params.objective.q_r config_data.mpc_params.objective.e_CL_exp config_data.mpc_params.objective.d_max config_data.mpc_params.objective.beta_steer config_data.mpc_params.objective.lambda config_data.mpc_params.objective.v_max config_data.mpc_params.objective.q_v_max config_data.model_params.l_f config_data.model_params.l_r config_data.model_params.m config_data.model_params.I_z config_data.model_params.T_max_front config_data.model_params.T_max_rear config_data.model_params.T_brake_front config_data.model_params.T_brake_rear config_data.model_params.GR config_data.model_params.eta_motor config_data.model_params.r_wheel config_data.model_params.g config_data.model_params.C_roll config_data.model_params.rho config_data.model_params.lambda_blend_min config_data.model_params.lambda_blend_max config_data.model_params.C_d config_data.model_params.C_l config_data.tyre_params.B config_data.tyre_params.C config_data.tyre_params.D config_data.model_params.downforce_front config_data.model_params.downforce_rear config_data.mpc_params.objective.beta_throttle config_data.model_params.h_cog config_data.tyre_params.mu_x config_data.tyre_params.mu_y config_data.mpc_params.ineq_limits.e_CL_max config_data.mpc_params.ineq_limits.delta_s_max config_data.mpc_params.ineq_limits.throttle_max config_data.mpc_params.ineq_limits.steering_max config_data.mpc_params.ineq_limits.delta_throttle_max config_data.mpc_params.ineq_limits.delta_steering_max config_data.model_params.width config_data.model_params.length config_data.mpc_params.ineq_limits.safety_margin config_data.mpc_params.objective.beta_epsilon config_data.mpc_params.objective.N config_data.controller_freq config_data.NN_config.NN config_data.mpc_params.objective.p config_data.mpc_params.objective.beta_psi config_data.model_params.track_width config_data.model_params.diff_gain ...
        s(n_points) NNparams coefs]';
    problem.all_parameters = repmat(parameters,N,1);
    problem.solver_timeout = config_data.solver_timeout;
    problem.num_of_threads = config_data.num_of_threads;

    % Time to solve the NLP!
    [output,exitflag,info] = solverMPCC(problem);
    % Make sure the solver has exited properly.
%     assert(exitflag == 1 || exitflag == 0,'Some problem in FORCES solver');
    fprintf('\nFORCES took %d iterations and %f seconds to solve the problem. Exit Flag: %d\n',info.it,info.solvetime, exitflag);
   

    % Debug information
    resizedZ = reshape(output.z,[config_data.mpc_params.objective.z config_data.mpc_params.objective.N-2]);
        
    i = 1;
    if STORE_MEX_DATA
        objective = zeros(N,1);
        jacObjectiveInit = zeros(config_data.mpc_params.objective.z-1,1);
        jacObjective = zeros(config_data.mpc_params.objective.z,N-2,1);
        jacObjectiveN = zeros(config_data.mpc_params.objective.z-4,1);
        sumObjective = zeros(1,1);
        % Ineq function
        ineqInit = zeros(4,1);
        ineq = zeros(6,N-2,1);
        ineqN = zeros(2,1);
        jacIneqInit = zeros(4,config_data.mpc_params.objective.z-1,1);
        jacIneq = zeros(6,config_data.mpc_params.objective.z,N-2,1);
        jacIneqN = zeros(2,config_data.mpc_params.objective.z-4,1);
        % Dynamics function
        dynamicsInit = zeros(10,1);
        dynamics = zeros(10,N-3,1);
        dynamicsN = zeros(8,1);
        jacDynamicsInit = zeros(10,config_data.mpc_params.objective.z-1,1);
        jacDynamics = zeros(10,config_data.mpc_params.objective.z,N-2,1);
        jacDynamicsN = zeros(8,config_data.mpc_params.objective.z,1);
        % Store the objective funtion values and jacobians
        [objective(1,i), jacObjectiveInit(:,i)] = solverMPCC_objective(output.zI,parameters,1);
        for j=1:N-2
            [objective(j+1,i), jacObjective(:,j,i)] = solverMPCC_objective(resizedZ(:,j),parameters,j+1);
        end
        [objective(N,i), jacObjectiveN(:,i)] = solverMPCC_objective(output.zN,parameters,N);
        sumObjective(i) = sum(objective(:,i));

        % Store non linear ineq values and jacobians
        [ineqInit(:,i), jacIneqInit(:,:,i)] = solverMPCC_ineq(output.zI,parameters,1);
        for j=1:N-2
            [ineq(:,j,i), jacIneq(:,:,j,i)] = solverMPCC_ineq(resizedZ(:,j),parameters,j+1);
        end
        [ineqN(:,i), jacIneqN(:,:,i)] = solverMPCC_ineq(output.zN,parameters,N);

        % Store equality values and jacobians
        [dynamicsInit(:,i), jacDynamicsInit(:,:,i)] = solverMPCC_dynamics(output.zI,parameters,1);
        for j=1:N-3
            [dynamics(:,j,i), jacDynamics(:,:,j,i)] = solverMPCC_dynamics(resizedZ(:,j),parameters,j+1);
        end
        [dynamicsN(:,i), jacDynamicsN(:,:,i)] = solverMPCC_dynamics(resizedZ(:,j+1),parameters,N-1);
    end

     %% Track
    interval = config_data.mpc_params.objective.z;
    figure
    plot(x,y,'--')
    x_spline = fnval(pp_x,linspace(0,s(n_points),1000));
    y_spline = fnval(pp_y,linspace(0,s(n_points),1000));
    plot(x_spline,y_spline,':')
    hold on
    plot([output.z(11:interval:end)' output.zN_1(9) output.zN(2)],[output.z(12:interval:end)' output.zN_1(10) output.zN(3)]);
    axis equal
    %Graphs
    xaxis_input = 1:N-1;
    xaxis_states = 1:N;
    xaxis_rate = 2:N;
    figure
    tiledlayout(2,5)
    nexttile
    plot(xaxis_input, [output.z(3:interval:end)' output.zN_1(1)]);
    title("throttle")
    nexttile
    plot(xaxis_input, [output.z(4:interval:end)' output.zN_1(2)]);
    title("steering");
    nexttile
    plot(xaxis_states, [output.z(13:interval:end)' output.zN_1(11) output.zN(4)]);
    title("Psi");
    nexttile
    plot(xaxis_states,[output.z(14:interval:end)' output.zN_1(12) output.zN(5)]);
    title("v_x");
    nexttile
    plot(xaxis_states, [output.z(15:interval:end)' output.zN_1(13) output.zN(6)]);
    title("v_y");
    nexttile
    plot(xaxis_states, [output.z(16:interval:end)' output.zN_1(14) output.zN(7)]);
    title("r");
    nexttile
    plot(xaxis_states, [output.z(11:interval:end)' output.zN_1(9) output.zN(2)]);
    title("X");
    nexttile
    plot(xaxis_states, [output.z(12:interval:end)' output.zN_1(10) output.zN(3)]);
    title("Y");
    nexttile
    plot(xaxis_states, [output.z(10:interval:end)' output.zN_1(8) output.zN(1)]);
    title("s");
    
    
end

%% ----------   Full Test      ------------------------------------------
% Plot the whole race considering the MPC model is the car model
clear applied_inputs;
clear exit_flags;
clear states;
clear solve_time;
clear rate_applied_inputs;
clear counts;
clear resizedZ_fail;

if FULL_TEST == true

    travelled = 0;
    N = config_data.mpc_params.objective.N;

    % Set initial guess to start solver from (here, middle of upper and lower bound)
    x0i=[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]';
    x0iN_1 = [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]';
    x0iN=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]';
    x0=repmat(x0i,N-2,1);
    
    problem.x0 = [x0;x0iN_1;x0iN];
    
    % Set initial condition
%     problem.xinit = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]'; 
    problem.xinit = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]'; 

    % Parameters
    parameters = [config_data.mpc_params.objective.alpha_CL config_data.mpc_params.objective.alpha_L config_data.mpc_params.objective.q_r config_data.mpc_params.objective.e_CL_exp config_data.mpc_params.objective.d_max config_data.mpc_params.objective.beta_steer config_data.mpc_params.objective.lambda config_data.mpc_params.objective.v_max config_data.mpc_params.objective.q_v_max config_data.model_params.l_f config_data.model_params.l_r config_data.model_params.m config_data.model_params.I_z config_data.model_params.T_max_front config_data.model_params.T_max_rear config_data.model_params.T_brake_front config_data.model_params.T_brake_rear config_data.model_params.GR config_data.model_params.eta_motor config_data.model_params.r_wheel config_data.model_params.g config_data.model_params.C_roll config_data.model_params.rho config_data.model_params.lambda_blend_min config_data.model_params.lambda_blend_max config_data.model_params.C_d config_data.model_params.C_l config_data.tyre_params.B config_data.tyre_params.C config_data.tyre_params.D config_data.model_params.downforce_front config_data.model_params.downforce_rear config_data.mpc_params.objective.beta_throttle config_data.model_params.h_cog config_data.tyre_params.mu_x config_data.tyre_params.mu_y config_data.mpc_params.ineq_limits.e_CL_max config_data.mpc_params.ineq_limits.delta_s_max config_data.mpc_params.ineq_limits.throttle_max config_data.mpc_params.ineq_limits.steering_max config_data.mpc_params.ineq_limits.delta_throttle_max config_data.mpc_params.ineq_limits.delta_steering_max config_data.model_params.width config_data.model_params.length config_data.mpc_params.ineq_limits.safety_margin config_data.mpc_params.objective.beta_epsilon config_data.mpc_params.objective.N config_data.controller_freq config_data.NN_config.NN config_data.mpc_params.objective.p config_data.mpc_params.objective.beta_psi config_data.model_params.track_width config_data.model_params.diff_gain];
%     problem.all_parameters = repmat(parameters,N,1);
    problem.solver_timeout = config_data.solver_timeout;
    problem.num_of_threads = config_data.num_of_threads;

    save_index = 738;
    
    % Time to solve the NLP!
    [~, track_length, cones, centerline, ~] = spline_creation_no_boundaries_trackdrive(40,15);
    i = 1;
    index = 1;
    resizedZ_fail = [];
    while travelled < track_length
        index_prev = index;
        % Get path coeffs
        index = get_nearest_index(problem.xinit(9),problem.xinit(10),centerline.X,centerline.Y);
        travelled = travelled + sqrt((centerline.X(index)-centerline.X(index_prev))^2+(centerline.Y(index)-centerline.Y(index_prev))^2);
        
        j = 1;
        jj = 1;
        max_distance = 50;
        clear tmp_X;
        clear tmp_Y;
        clear tmp_progresses;
        while true
            if index + j > length(centerline.X)
                tmp_X(j) = centerline.X(jj);
                tmp_Y(j) = centerline.Y(jj);
                jj = jj + 1;
            else
                tmp_X(j) = centerline.X(index + j - 1);
                tmp_Y(j) = centerline.Y(index + j - 1);
            end
            
            if j == 1
                tmp_progresses(j) = 0;
            else
                tmp_progresses(j) = tmp_progresses(j-1) + sqrt((tmp_X(j)-tmp_X(j-1))^2 + (tmp_Y(j)-tmp_Y(j-1))^2);
            end

            if tmp_progresses(j) > max_distance
                break;
            end

            j = j + 1;
        end
        X_spline = spline(tmp_progresses,tmp_X);
        Y_spline = spline(tmp_progresses,tmp_Y);
        progresses = linspace(0,tmp_progresses(end),config_data.mpc_params.objective.spline);
        X = fnval(X_spline,progresses);
        Y = fnval(Y_spline,progresses);
        X_spline = spline(progresses,X);
        Y_spline = spline(progresses,Y);
        pp_x_coefs = flip(X_spline.coefs,2);
        pp_y_coefs = flip(Y_spline.coefs,2);
        
        n_points = config_data.mpc_params.objective.spline;
        for k = 1:n_points-1
            for kk = 1:4
                coefs((k-1)*4+kk) =  pp_x_coefs(k,kk);
            end
        end
        for k = 1:n_points-1
            for kk = 1:4
                coefs((n_points-1)*4+(k-1)*4+kk) =  pp_y_coefs(k,kk);
            end
        end
        parameters_final = [parameters progresses(end) NNparams coefs]';
        problem.all_parameters = repmat(parameters_final,N,1);
        % Come up with a solution and store
        [output,exit_flags(i),info] = solverMPCC(problem);
        applied_inputs(:,i) = output.z(4:5);
        states(:,i) = output.z(12:17);
        progress(:,i) = output.z(11);
        rate_applied_inputs(:,i) = output.z(1:2);
        solve_time(i) = info.solvetime;

        resizedZ = reshape(output.z,[config_data.mpc_params.objective.z config_data.mpc_params.objective.N-2]);
        zN_1 = output.zN_1;
        zN = output.zN;

        if i == save_index
            resizedZ_save = resizedZ;
            zN_1_save = zN_1;
            zN_save = zN;
        end

        if exit_flags(i) == 1
            resizedZ_fail = output.z;
            zN_1_fail = zN_1;
            zN_fail = zN;
        end

        interval = config_data.mpc_params.objective.z;

        if STORE_MEX_DATA
            % Store the objective funtion values and jacobians
%             [objective(1,i), jacObjectiveInit(:,i)] = solverMPCC_objective(output.zI,parameters,1);
%             for j=1:N-2
%                 [objective(j+1,i), jacObjective(:,j,i)] = solverMPCC_objective(resizedZ(:,j),parameters,j+1);
%             end
%             [objective(N,i), jacObjectiveN(:,i)] = solverMPCC_objective(output.zN,parameters,N);
%             sumObjective(i) = sum(objective(:,i));
% 
%             % Store non linear ineq values and jacobians
%             [ineqInit(:,i), jacIneqInit(:,:,i)] = solverMPCC_ineq(output.zI,parameters,1);
%             for j=1:N-2
%                 [ineq(:,j,i), jacIneq(:,:,j,i)] = solverMPCC_ineq(resizedZ(:,j),parameters,j+1);
%             end
%             [ineqN(:,i), jacIneqN(:,:,i)] = solverMPCC_ineq(output.zN,parameters,N);

            % Store equality values and jacobians
%             for j=1:N-2
%                 [dynamics(:,j,i), jacDynamics(:,:,j,i)] = solverMPCC_dynamics(resizedZ(:,j),parameters,j);
%             end
%             [dynamicsN_1(:,i), jacDynamicsN_1(:,:,i)] = solverMPCC_dynamics(zN_1,parameters,N-1);
        end
    
        % Set initial guess
        x0i=output.z;
        x0iN_1=output.zN_1;
        x0iN=output.zN;
    
        problem.x0 = [x0i;x0iN_1;x0iN];
    
        % xinit
        if (exit_flags(i) == 1 || isempty(resizedZ_fail))
%             problem.xinit = [output.z(3 + interval),output.z(4 + interval),output.z(3),output.z(4),output.z(14),output.z(15),output.z(16),0,output.z(11 + interval),output.z(12 + interval),output.z(13 + interval),output.z(14 + interval),output.z(15 + interval),output.z(16 + interval)]';
            fail_counter = 1;
            problem.xinit = [output.z(4 + interval*fail_counter),output.z(5 + interval*fail_counter),output.z(6 + interval*fail_counter),output.z(7 + interval*fail_counter),output.z(8 + interval*fail_counter),output.z(9 + interval*fail_counter),output.z(10 + interval*fail_counter),0,output.z(12 + interval*fail_counter),output.z(13 + interval*fail_counter),output.z(14 + interval*fail_counter),output.z(15 + interval*fail_counter),output.z(16 + interval*fail_counter),output.z(17 + interval*fail_counter)]';
            
        elseif fail_counter < 20
            fail_counter = fail_counter + 1;
            problem.xinit = [resizedZ_fail(4 + interval*fail_counter),resizedZ_fail(5 + interval*fail_counter),resizedZ_fail(6 + interval*fail_counter),resizedZ_fail(7 + interval*fail_counter),resizedZ_fail(8 + interval*fail_counter),resizedZ_fail(9 + interval*fail_counter),resizedZ_fail(10 + interval*fail_counter),0,resizedZ_fail(12 + interval*fail_counter),resizedZ_fail(13 + interval*fail_counter),resizedZ_fail(14 + interval*fail_counter),resizedZ_fail(15 + interval*fail_counter),resizedZ_fail(16 + interval*fail_counter),resizedZ_fail(17 + interval*fail_counter)]';
        else
            exit('Not good')
        end
            i = i + 1;
    end

    if STORE_MEX_DATA
        % Save a part for testing purposes
        test.vx = [resizedZ_save(14,1) dynamics(12,:,save_index)];
        test.vy = [resizedZ_save(15,1) dynamics(13,:,save_index)];
        test.r = [resizedZ_save(16,1) dynamics(14,:,save_index)];
    
        test.vx_prev = [resizedZ_save(7,1) dynamics(5,:,save_index)];
        test.vy_prev = [resizedZ_save(8,1) dynamics(6,:,save_index)];
        test.r_prev = [resizedZ_save(9,1) dynamics(7,:,save_index)];
    
        test.throttles = [resizedZ_save(3,1) dynamics(1,:,save_index)];
        test.steerings = [resizedZ_save(4,1) dynamics(2,:,save_index)];
    
        test.throttles_prev = [resizedZ_save(5,1) dynamics(3,:,save_index)];
        test.steerings_prev = [resizedZ_save(6,1) dynamics(4,:,save_index)];

        test.vxN = dynamicsN_1(5,save_index);
        test.vyN = dynamicsN_1(6,save_index);
        test.rN = dynamicsN_1(7,save_index);

        test.resizedZ = resizedZ_save;
        test.zN_1 = zN_1_save;
        test.zN = zN_save;
    
%         save('../AI/scripts/test_dynamics.mat','test');
    end

    one_count = sum(exit_flags(:) == 1);
    two_count = sum(exit_flags(:) == 2);
    six_count = sum(exit_flags(:) == -6);
    seven_count = sum(exit_flags(:) == -7);
    other_count = length(exit_flags)- one_count - two_count - six_count - seven_count;
    counts = [one_count two_count six_count seven_count other_count];
    avg_solve_time = sum(solve_time)/length(solve_time);
    fprintf("Convergence rate: %f%%\n",(one_count/length(exit_flags))*100);
    
    %% Track positions and states/inputs plots

    track_layout(1,:) = centerline.X;
    track_layout(2,:) = centerline.Y;
 
    figure

    plot(track_layout(1,:),track_layout(2,:),'--');
    hold on
    scatter(cones.blue(:,1),cones.blue(:,2), 10,[0 0.4470 0.7410], 'filled');
    scatter(cones.yellow(:,1),cones.yellow(:,2), 10,[0.9290 0.6940 0.1250], 'filled');

    n = length(states(1,:));
    cm = jet(n);
    cm = flip(cm);
    colormap(cm);

    min_vel = 0;
    max_vel = 22;
        
    vel = sqrt(states(4,:).^2+states(5,:).^2);
    ind = interp1([min_vel max_vel], [1 n], vel);
    ind = round(ind);
    cm = cm(ind,:);

    for i = 1:n-1
        j = i:i+1;
        plot(states(1,j),states(2,j),'color',cm(i,:),'LineWidth',1.5)
    end
    caxis([min_vel max_vel]);
    colorbar
    axis equal
    hold off
    
    %% Graphs
    xaxis_input = 1:length(applied_inputs(1,:));
    xaxis_states = 1:length(states(1,:));
    xaxis_rate = 2:length(rate_applied_inputs(1,:));
    figure
    tiledlayout(2,7)
    nexttile
    plot(xaxis_input, applied_inputs(1,:));
    hold on
    yline(config_data.mpc_params.ineq_limits.throttle_min);
    yline(config_data.mpc_params.ineq_limits.throttle_max);
    hold off
    title("throttle")
    nexttile
    plot(xaxis_input, applied_inputs(2,:));
    hold on
    yline(config_data.mpc_params.ineq_limits.steering_min);
    yline(config_data.mpc_params.ineq_limits.steering_max);
    hold off
    title("steering");
    nexttile
    plot(xaxis_states, states(3,:));
    title("\Psi");
    nexttile
    plot(xaxis_states,states(4,:));
    title("v_x");
    nexttile
    plot(xaxis_states,states(5,:));
    title("v_y");
    nexttile
    plot(xaxis_states, states(6,:));
    title("r");
    aprox_inv_velocity = 1./(states(4,:)+2./(1+states(4,:)));
    yaw_rate_limit = 0.925*config_data.tyre_params.mu_x*config_data.model_params.g.*aprox_inv_velocity;
    plot(xaxis_states, states(6,:));
    hold on
    plot(xaxis_states, yaw_rate_limit)
    plot(xaxis_states,-yaw_rate_limit)
    nexttile
    plot(xaxis_states,  states(1,:));
    title("X");
    nexttile
    plot(xaxis_states,  states(2,:));
    title("Y");
    nexttile
    plot(xaxis_states, rate_applied_inputs(1,:));
    hold on
    yline(-config_data.mpc_params.ineq_limits.delta_throttle_max);
    yline(config_data.mpc_params.ineq_limits.delta_throttle_max);
    hold off
    title("Throttle Rate");
    nexttile
    plot(xaxis_states, rate_applied_inputs(2,:));
    hold on
    yline(-config_data.mpc_params.ineq_limits.delta_steering_max);
    yline(config_data.mpc_params.ineq_limits.delta_steering_max);
    hold off
    title("Steering Rate");
    nexttile
    plot(xaxis_states, exit_flags);
    title("exit flag");
    nexttile
    plot(xaxis_states, solve_time);
    hold on
    yline(config_data.solver_timeout);
    hold off
    title("solve time");
    nexttile
    histogram(solve_time,20);
    title("solve time histogram");
end


%% ----------   Change MatLab Files    ----------------------------------

if REPLACE_MATLAB
    disp("Changing headers and organizing files within MATLAB folder");
    % Replace adtool2forces includes
    find_and_replace('solverMPCC_adtool2forces.c','\<#include "solverMPCC/include/solverMPCC.h"\>','#include "include/solverMPCC.h"');
%     find_and_replace('solverMPCC_adtool2forces.c','\<#include "solverMPCC_casadi.h"\>',['#include "solverMPCC_casadi.h"' newline '#include "solverMPCC_casadi.c"']);
    
    % move files to output folder
    copyfile("solverMPCC_casadi.c",folder_name);
    copyfile("solverMPCC_casadi.h",folder_name);
    copyfile("solverMPCC_adtool2forces.c",folder_name);
    copyfile('solverMPCC/include/', [folder_name '/include/']);
    copyfile('solverMPCC/lib/', [folder_name '/lib/']);
    
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
%     copyfile("solverMPCC",rest_folder_name);
    movefile(rest_folder_name,folder_name);

else
    disp("NOT Changing headers and organizing files");
end
%% ----------   Change Cpp Files    -------------------------------------
if REPLACE_CPP
    disp("Replacing Cpp files");
    system('rm -r ../solverLMPC/');
    copyfile(folder_name,'../solverLMPC');
else
    disp("NOT Replacing Cpp files");
end



%% ----------   Finalization        -------------------------------------

disp(['------------------------------------------------------',newline ,...
    'Output files will be in a folder named "', folder_name, '"'])

function index = get_nearest_index(X,Y,path_X,path_Y)
    min_distance = inf;
    index = 1;
    for i = 1:length(path_X)
        distance = sqrt((X-path_X(i))^2+(Y-path_Y(i))^2);
        if distance <= min_distance
            min_distance = distance;
            index = i;
        end
    end
end