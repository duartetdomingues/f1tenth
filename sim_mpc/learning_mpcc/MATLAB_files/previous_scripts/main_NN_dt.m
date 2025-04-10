% This script generates the necessary files to use in the MPC controller
%
% Developped by : David Gomes, November 2022,
% david.m.r.gomes@tecnico.ulisboa.pt
% Kudos to Grabriel Costa, gafc@live.com.pt, for the initial ideia

%% ----------   Initialization  -----------------------------------------
clear all; clc; close all
% Add yaml parser to path
addpath("yaml_parser/")

% Downsample of centerline for spline
SPLINE_POINTS = 35;
EXTENDED_POINTS = 15; % Zero is currently not supported

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

%% ----------   Select config file   ------------------------------------

% [config_file,config_path] = uigetfile('../config/*.yaml', 'Select a config file');
% config_data =  yaml.loadFile([config_path,config_file]);
config_data =  yaml.loadFile('../config/default.yaml');

%% ----------   Generate Splines  ---------------------------------------

[matSpline, track_length, folder_name] = spline_creation(SPLINE_POINTS,EXTENDED_POINTS);

%% ----------   Generate NN Model  --------------------------------------

modelName = strcat('model_',int2str(config_data.NN_config.model_id));
NNstruct = get_NN_params_torch(modelName,1);
% NNstruct = get_NN_params(modelName,T);
NNparams = [NNstruct.fc1_kernel(:)' NNstruct.fc1_bias(:)' NNstruct.fc2_kernel(:)' NNstruct.fc2_bias(:)' NNstruct.rnn_kernel(:)' NNstruct.rnn_recurrent_kernel(:)' NNstruct.rnn_bias(:)' NNstruct.rnn_recurrent_bias(:)' NNstruct.fc2_rev_kernel(:)' NNstruct.fc2_rev_bias(:)' NNstruct.fc1_rev_kernel(:)' NNstruct.fc1_rev_bias(:)' NNstruct.fc_out_kernel(:)' NNstruct.fc_out_bias(:)'];

%% ----------   Generate FORCESPRO Solver   -----------------------------
if GENERATE_SOLVER
    extended_distance = track_length/SPLINE_POINTS*EXTENDED_POINTS;
%     formulateMPCC_NN_dt(matSpline, config_data, track_length, extended_distance,STORE_MEX_DATA);
%     formulateMPCC_noNNandKin_dt(matSpline, config_data, track_length, extended_distance,STORE_MEX_DATA);
    formulateMPCC_noNN_dt(matSpline, config_data, track_length, extended_distance,STORE_MEX_DATA);
end

%% ----------   UNIT TESTING --------------------------------------------

if UNIT_TEST == true

     N = config_data.mpc_params.objective.N;

    % Set initial guess to start solver from (here, middle of upper and lower bound)
    x0iI=[0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]';
    x0iM=[0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]';
    x0iZ=[0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]';
    x0=repmat(x0iM,N-2,1);
    
    problem.x0 = [x0iI;x0;x0iZ];
    
    % Set initial condition
    % Initial position of fssim
%     problem.xinit = [0,0,0,0,0,0,0,0,0]'; 
%     problem.xinit = [0,0,0,0,1.50,0,0,0,0,0,0]'; 
    problem.xinit = [0,0,0,0,0,0,0,0,0,0,0]'; 

    % Parameters
    parameters = [config_data.mpc_params.objective.alpha_CL config_data.mpc_params.objective.alpha_L config_data.mpc_params.objective.q_r config_data.mpc_params.objective.e_CL_exp config_data.mpc_params.objective.d_max config_data.mpc_params.objective.beta_steer config_data.mpc_params.objective.lambda config_data.mpc_params.objective.v_max config_data.mpc_params.objective.q_v_max config_data.model_params.l_f config_data.model_params.l_r config_data.model_params.m config_data.model_params.I_z config_data.model_params.T_max_front config_data.model_params.T_max_rear config_data.model_params.T_brake_front config_data.model_params.T_brake_rear config_data.model_params.GR config_data.model_params.eta_motor config_data.model_params.r_wheel config_data.model_params.g config_data.model_params.C_roll config_data.model_params.rho config_data.model_params.lambda_blend_min config_data.model_params.lambda_blend_max config_data.model_params.C_d config_data.model_params.C_l config_data.tyre_params.B config_data.tyre_params.C config_data.tyre_params.D config_data.model_params.downforce_front config_data.model_params.downforce_rear config_data.mpc_params.objective.beta_throttle config_data.model_params.h_cog config_data.tyre_params.mu_x config_data.tyre_params.mu_y config_data.mpc_params.ineq_limits.e_CL_max config_data.mpc_params.ineq_limits.delta_s_max config_data.mpc_params.ineq_limits.throttle_max config_data.mpc_params.ineq_limits.steering_max config_data.mpc_params.ineq_limits.delta_throttle_max config_data.mpc_params.ineq_limits.delta_steering_max config_data.mpc_params.objective.beta_epsilon config_data.model_params.width config_data.model_params.length config_data.mpc_params.ineq_limits.safety_margin]';
%     parameters = [config_data.mpc_params.objective.alpha_CL config_data.mpc_params.objective.alpha_L config_data.mpc_params.objective.q_r config_data.mpc_params.objective.e_CL_exp config_data.mpc_params.objective.d_max config_data.mpc_params.objective.beta_steer config_data.mpc_params.objective.lambda config_data.mpc_params.objective.v_max config_data.mpc_params.objective.q_v_max config_data.model_params.l_f config_data.model_params.l_r config_data.model_params.m config_data.model_params.I_z config_data.model_params.T_max_front config_data.model_params.T_max_rear config_data.model_params.T_brake_front config_data.model_params.T_brake_rear config_data.model_params.GR config_data.model_params.eta_motor config_data.model_params.r_wheel config_data.model_params.g config_data.model_params.C_roll config_data.model_params.rho config_data.model_params.lambda_blend_min config_data.model_params.lambda_blend_max config_data.model_params.C_d config_data.model_params.C_l config_data.tyre_params.B config_data.tyre_params.C config_data.tyre_params.D config_data.model_params.downforce_front config_data.model_params.downforce_rear config_data.mpc_params.objective.beta_throttle config_data.model_params.h_cog config_data.tyre_params.mu_x config_data.tyre_params.mu_y]';
    problem.all_parameters = repmat(parameters,N,1);
    problem.solver_timeout = config_data.solver_timeout;
    problem.num_of_threads = config_data.num_of_threads;

    % Time to solve the NLP!
    [output,exitflag,info] = solverMPCC(problem);

    resizedZ = reshape(output.z,[config_data.mpc_params.objective.z config_data.mpc_params.objective.N-2]);
    if STORE_MEX_DATA
        % Store the objective funtion values and jacobians
        [objective(1), jacObjectiveInit(:,1)] = solverMPCC_objective(output.zI,parameters,1);
        for j=1:N-2
            [objective(j+1), jacObjective(:,j)] = solverMPCC_objective(resizedZ(:,j),parameters,j+1);
        end
        [objective(N), jacObjectiveN] = solverMPCC_objective(output.zN,parameters,N);
        sumObjective = sum(objective);

        % Store non linear ineq values and jacobians
        [ineqInit, jacIneqInit] = solverMPCC_ineq(output.zI,parameters,1);
        for j=1:N-2
            [ineq(:,j), jacIneq(:,:,j)] = solverMPCC_ineq(resizedZ(:,j),parameters,j+1);
        end
        [ineqN, jacIneqN] = solverMPCC_ineq(output.zN,parameters,N);

        % Store equality values and jacobians
        [dynamicsInit, jacDynamicsInit] = solverMPCC_dynamics(output.zI,parameters,1);
        for j=1:N-3
            [dynamics(:,j), jacDynamics(:,:,j)] = solverMPCC_dynamics(resizedZ(:,j),parameters,j+1);
        end
        [dynamicsN, jacDynamicsN] = solverMPCC_dynamics(resizedZ(:,j+1),parameters,N-1);
    end

    % Make sure the solver has exited properly.
    assert(exitflag == 1 || exitflag == 0 || exitflag == 2,'Some problem in FORCES solver');
    fprintf('\nFORCES took %d iterations and %f seconds to solve the problem.\n',info.it,info.solvetime);
    
    %% Track
    if GENERATE_SOLVER
    track_layout = zeros(2,ceil(track_length)+1);
        for i=1:ceil(track_length)
            [track_layout(1,i),track_layout(2,i),~,~] = splineTrack(i-1);
        end
        [track_layout(1,ceil(track_length)+1),track_layout(2,ceil(track_length)+1),~,~] = splineTrack(track_length);
    else
        track_layout(1,:) = centerline.X;
        track_layout(2,:) = centerline.Y;
    end

    figure
    plot([output.zI(6) output.z(7:12:end)' output.zN(3)],[output.zI(7) output.z(8:12:end)' output.zN(4)]);
    hold on
    plot(track_layout(1,:),track_layout(2,:),'--');
    hold off
    
    %Graphs
    xaxis_input = 1:N-1;
    xaxis_states = 1:N;
    xaxis_rate = 2:N;
    figure
    tiledlayout(2,5)
    nexttile
    plot(xaxis_input, [output.zI(1) output.z(1:12:end)']);
    title("throttle")
    nexttile
    plot(xaxis_input, [output.zI(2) output.z(2:12:end)']);
    title("steering");
    nexttile
    plot(xaxis_states, [output.zI(8) output.z(9:12:end)' output.zN(5)]);
    title("Psi");
    nexttile
    plot(xaxis_states,[output.zI(9) output.z(10:12:end)' output.zN(6)]);
    title("v_x");
    nexttile
    plot(xaxis_states, [output.zI(10) output.z(11:12:end)' output.zN(7)]);
    title("v_y");
    nexttile
    plot(xaxis_states, [output.zI(11) output.z(12:12:end)' output.zN(8)]);
    title("r");
    nexttile
    plot(xaxis_states, [output.zI(6) output.z(7:12:end)' output.zN(3)]);
    title("X");
    nexttile
    plot(xaxis_states, [output.zI(7) output.z(8:12:end)' output.zN(4)]);
    title("Y");
    nexttile
    plot(xaxis_states, [output.zI(5) output.z(6:12:end)' output.zN(2)]);
    title("s");
    nexttile
    plot(xaxis_rate, [output.z(6:12:end)' output.zN(2)]);
    title("s_{t-1}");
    
    
    % Plotting track progresses
%     track_layout = zeros(2,310);
%     
%     figure
%     hold on
%     plot(track_layout(1,:),track_layout(2,:));
%     plot(track_data.X,track_data.Y);
%     hold off
end

%% ----------   FULL TEST       -----------------------------------------
% Plot the whole race considering the MPC model is the car model
clear applied_inputs exit_flags states solve_time rate_applied_inputs progress

if FULL_TEST == true

    duration = 800;
    N = config_data.mpc_params.objective.N;

    % Set initial guess to start solver from (here, middle of upper and lower bound)
    x0i=[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]';
    x0iN_1 = [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]';
    x0iN=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]';
    x0=repmat(x0i,N-2,1);
    
    problem.x0 = [x0;x0iN_1;x0iN];
    
    % Set initial condition
%     problem.xinit = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]'; 
    problem.xinit = [0,0,0,0,0,0,0,1.50,0,0,0,0,0,0]'; 
%     problem.xinit = [0,0,0,0,0,0,0,0,-1.49,-0.482,0,0,0,0]'; 
    
    % Parameters
    parameters = [config_data.mpc_params.objective.alpha_CL config_data.mpc_params.objective.alpha_L config_data.mpc_params.objective.q_r config_data.mpc_params.objective.e_CL_exp config_data.mpc_params.objective.d_max config_data.mpc_params.objective.beta_steer config_data.mpc_params.objective.lambda config_data.mpc_params.objective.v_max config_data.mpc_params.objective.q_v_max config_data.model_params.l_f config_data.model_params.l_r config_data.model_params.m config_data.model_params.I_z config_data.model_params.T_max_front config_data.model_params.T_max_rear config_data.model_params.T_brake_front config_data.model_params.T_brake_rear config_data.model_params.GR config_data.model_params.eta_motor config_data.model_params.r_wheel config_data.model_params.g config_data.model_params.C_roll config_data.model_params.rho config_data.model_params.lambda_blend_min config_data.model_params.lambda_blend_max config_data.model_params.C_d config_data.model_params.C_l config_data.tyre_params.B config_data.tyre_params.C config_data.tyre_params.D config_data.model_params.downforce_front config_data.model_params.downforce_rear config_data.mpc_params.objective.beta_throttle config_data.model_params.h_cog config_data.tyre_params.mu_x config_data.tyre_params.mu_y config_data.mpc_params.ineq_limits.e_CL_max config_data.mpc_params.ineq_limits.delta_s_max config_data.mpc_params.ineq_limits.throttle_max config_data.mpc_params.ineq_limits.steering_max config_data.mpc_params.ineq_limits.delta_throttle_max config_data.mpc_params.ineq_limits.delta_steering_max config_data.model_params.width config_data.model_params.length config_data.mpc_params.ineq_limits.safety_margin config_data.mpc_params.objective.beta_epsilon config_data.mpc_params.objective.N config_data.controller_freq config_data.NN_config.NN config_data.mpc_params.objective.p config_data.mpc_params.objective.beta_psi NNparams]';
    problem.all_parameters = repmat(parameters,N,1);
    problem.solver_timeout = config_data.solver_timeout;
    problem.num_of_threads = config_data.num_of_threads;
    % Time to solve the NLP!
    i = 1;
    while problem.xinit(8) < track_length
        % Come up with a solution and store
        [output,exit_flags(i),info] = solverMPCC(problem);
        applied_inputs(:,i) = output.z(3:4);
    %     states(:,i) = output.z(7:12);
    %     progress(:,i) = output.z(6);
        states(:,i) = output.z(11:16);
        progress(:,i) = output.z(10);
        rate_applied_inputs(:,i) = output.z(1:2);
        solve_time(i) = info.solvetime;
        resizedZ = reshape(output.z,[config_data.mpc_params.objective.z config_data.mpc_params.objective.N-2]);
        
        interval = config_data.mpc_params.objective.z;

        if STORE_MEX_DATA
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
    
        % Set initial guess
        x0i=output.z;
        x0iN_1=output.zN_1;
        x0iN=output.zN;
    
        problem.x0 = [x0i;x0iN_1;x0iN];
    
        % xinit
        problem.xinit = [output.z(3 + interval),output.z(4 + interval),output.z(3),output.z(4),output.z(14),output.z(15),output.z(16),output.z(10 + interval),output.z(11 + interval),output.z(12 + interval),output.z(13 + interval),output.z(14 + interval),output.z(15 + interval),output.z(16 + interval)]';
        i = i + 1;
    end

%     if STORE_MEX_DATA
%         % Store the objective funtion values and jacobians
%         [objective, jacObjectiveInit] = solverMPCC_objective(output.zI,parameters,1);
%         for j=1:N-2
%             [objective(j+1), jacObjective(:,j)] = solverMPCC_objective(resizedZ(:,j),parameters,j+1);
%         end
%         [objective(N), jacObjectiveN] = solverMPCC_objective(output.zN,parameters,N);
%         sumObjective = sum(objective);
% 
%         % Store non linear ineq values and jacobians
%         [ineqInit, jacIneqInit] = solverMPCC_ineq(output.zI,parameters,1);
%         for j=1:N-2
%             [ineq(:,j), jacIneq(:,:,j)] = solverMPCC_ineq(resizedZ(:,j),parameters,j+1);
%         end
%         [ineqN, jacIneqN] = solverMPCC_ineq(output.zN,parameters,N);
% 
%         % Store equality values and jacobians
%         [dynamicsInit, jacDynamicsInit] = solverMPCC_dynamics(output.zI,parameters,1);
%         for j=1:N-3
%             [dynamics(:,j), jacDynamics(:,:,j)] = solverMPCC_dynamics(resizedZ(:,j),parameters,j+1);
%         end
%         [dynamicsN, jacDynamicsN] = solverMPCC_dynamics(resizedZ(:,j+1),parameters,N-1);
%     end
%     lastNonzeroIndex = find(states(1,:) ~= 0, 1, 'last');
%     states(1,:) = states(1,1:lastNonzeroIndex);
%     lastNonzeroIndex = find(states(2,:) ~= 0, 1, 'last');
%     states(2,:) = states(2,1:lastNonzeroIndex);
%     lastNonzeroIndex = find(states(3,:) ~= 0, 1, 'last');
%     states(3,:) = states(3,1:lastNonzeroIndex);
%     lastNonzeroIndex = find(states(4,:) ~= 0, 1, 'last');
%     states(4,:) = states(4,1:lastNonzeroIndex);
%     lastNonzeroIndex = find(states(5,:) ~= 0, 1, 'last');
%     states(5,:) = states(5,1:lastNonzeroIndex);
%     lastNonzeroIndex = find(states(6,:) ~= 0, 1, 'last');
%     states(6,:) = states(6,1:lastNonzeroIndex);
%     lastNonzeroIndex = find(applied_inputs(1,:) ~= 0, 1, 'last');
%     applied_inputs(1,:) = applied_inputs(1,1:lastNonzeroIndex);
%     lastNonzeroIndex = find(applied_inputs(2,:) ~= 0, 1, 'last');
%     applied_inputs(2,:) = applied_inputs(2,1:lastNonzeroIndex);
%     lastNonzeroIndex = find(exit_flags ~= 0, 1, 'last');
%     exit_flags = exit_flags(1:lastNonzeroIndex);
%     lastNonzeroIndex = find(rate_applied_inputs(1,:) ~= 0, 1, 'last');
%     rate_applied_inputs(1,:) = rate_applied_inputs(1,1:lastNonzeroIndex);
%     lastNonzeroIndex = find(rate_applied_inputs(2,:) ~= 0, 1, 'last');
%     rate_applied_inputs(2,:) = rate_applied_inputs(2,1:lastNonzeroIndex);


    one_count = sum(exit_flags(:) == 1);
    two_count = sum(exit_flags(:) == 2);
    six_count = sum(exit_flags(:) == -6);
    seven_count = sum(exit_flags(:) == -7);
    other_count = length(exit_flags)- one_count - two_count - six_count - seven_count;
    counts = [one_count two_count six_count seven_count other_count];
    avg_solve_time = sum(solve_time)/length(solve_time);
    
% sum(A(:) == 1);
    %% Track positions and states/inputs plots
    
    if GENERATE_SOLVER
        track_layout = zeros(2,ceil(track_length));
        for i=1:ceil(track_length)
            track_layout(1,i) = fnval(matSpline.xSpline,i-1);
            track_layout(2,i) = fnval(matSpline.ySpline,i-1);
        end
    else
        track_layout(1,:) = centerline.X;
        track_layout(2,:) = centerline.Y;
    end

    figure

    plot(track_layout(1,:),track_layout(2,:),'--');
    hold on

    n = length(states(1,:));
    cm = jet(n);
    cm = flip(cm);
    colormap(cm);

    min_vel = 0;
    max_vel = 20;
        
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
    hold off
    
    %% Graphs
%     xaxis_input = 1:duration;
%     xaxis_states = 1:duration;
%     xaxis_rate = 2:duration;
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
%     plot(xaxis_states, ones(1,duration)*config_data.mpc_params.ineq_limits.throttle_min);
%     plot(xaxis_states, ones(1,duration)*config_data.mpc_params.ineq_limits.throttle_max);
    hold off
    title("throttle")
    nexttile
    plot(xaxis_input, applied_inputs(2,:));
    hold on
    yline(config_data.mpc_params.ineq_limits.steering_min);
    yline(config_data.mpc_params.ineq_limits.steering_max);
%     plot(xaxis_states, ones(1,duration)*config_data.mpc_params.ineq_limits.steering_min);
%     plot(xaxis_states, ones(1,duration)*config_data.mpc_params.ineq_limits.steering_max);
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
    aprox_inv_velocity = 1./(states(4,:)+0.05./(1+states(4,:)));
    yaw_rate_limit = 0.85*config_data.tyre_params.mu_x*config_data.model_params.g.*aprox_inv_velocity;
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
    plot(xaxis_states, progress);
    title("s");
    nexttile
    plot(xaxis_states, rate_applied_inputs(1,:));
    hold on
    yline(-config_data.mpc_params.ineq_limits.delta_throttle_max);
    yline(config_data.mpc_params.ineq_limits.delta_throttle_max);
%     plot(xaxis_states, ones(1,duration)*config_data.mpc_params.ineq_limits.delta_throttle_min);
%     plot(xaxis_states, ones(1,duration)*config_data.mpc_params.ineq_limits.delta_throttle_max);
    hold off
    title("Throttle Rate");
    nexttile
    plot(xaxis_states, rate_applied_inputs(2,:));
    hold on
    yline(-config_data.mpc_params.ineq_limits.delta_steering_max);
    yline(config_data.mpc_params.ineq_limits.delta_steering_max);
%     plot(xaxis_states, ones(1,duration)*config_data.mpc_params.ineq_limits.delta_steering_min);
%     plot(xaxis_states, ones(1,duration)*config_data.mpc_params.ineq_limits.delta_steering_max);
    hold off
    title("Steering Rate");
    nexttile
    plot(xaxis_states, exit_flags);
    title("exit flag");
    nexttile
    plot(xaxis_states, solve_time);
    hold on
    yline(config_data.solver_timeout);
%     plot(xaxis_states, ones(1,duration)*config_data.solver_timeout);
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

    % update track length in the config file
    config_data.track_length = track_length;
    yaml.dumpFile('../config/default.yaml',config_data);

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