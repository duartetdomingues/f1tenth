%% Speed and Steering Control for Autonomous Electric Vehicles
% Fábio Portelinha  2021/2022   IST
%
%   Supervisors:    José Gaspar
%                   João Fernandes
%

%% Clear all is needed in order to consecutively run this implementation
%clear all 
%% Global Variables definition
globalDef()
load('vehicleParameters.mat')
global t_step L STEER_MAX ACC_MAX SPEED_MAX SPEED_MIN GOAL_DIST MAX_T N_X N_U MAX_ITER;
global d_U_MAX DU_TH STOP_SPEED SEARCH_RANGE MIN_COST POSE_ITER Kp;
global A gl M rw Cd gr Iw Im rho vw Crr1 Crr2 pp
%% ACADOS Implementation 
addpath('~/Desktop/MPC/acados/examples/acados_matlab_octave/')

%acados_env_variables_windows()

check_acados_requirements()
import casadi.*
g=9.81;


% Pick path and pitch
% Available paths: '13_04_21', 'long_line', 'lin_ret', 'line2', 'sin',
% '05_05_21' -> sharp corner 
% '06_05_21' -> u shaped
% '11_05_21' -> u shaped even more
% Available pitch: 'flat', 'uphill5', 'uphill10', 'downhill5', 'downhill10', 'updown'

path_tag = '13_04_21';
pitch_tag = 'flat';
ref_path = input_scenario_V3(path_tag);
ref_path = add_path_theta(ref_path, pitch_tag);

% Create or load speed profile
sp_tag = strcat('sp/', path_tag, '_', pitch_tag, '.mat');
if(exist(sp_tag, 'file'))
    load(sp_tag)
else
    tic
    [sp,t_sp, E] = sp_20_06_21(ref_path, 1, 0.1, inf);
    save(sp_tag, 'sp', 't_sp', 'E')
    toc
end

% Extract x and y coordinates from the path
X_viena(:, 1) =  ref_path(:,1);
X_viena(:, 2) =  ref_path(:,2);

% Calculate curvature of the path
[~, R_viena, ~] = curvature(X_viena);

% checkSlip(ref_path,sp)
%% Solver parameters
torpcompile_interface = 'auto';
codgen_model = 'true';
nlp_solver = 'sqp_rti';
qp_solver = 'partial_condensing_hpipm';
nlp_solver_exact_hessian = 'false'; 
qp_solver_cond_N = 5; 
regularize_method = 'no_regularize';
sim_method = 'erk'; 

%% horizon parameters
N = 5; % Number of prediction horizon timesteps
T = 1.0; % Prediction horizon length [s]

%% model dynamics
[model, constraint] = car_model; % vehicle dynamics and mechanical constraints
nx = length(model.x); % State Vector Length
nu = length(model.u); % Control vector Length

%% model to create the solver
ocp_model = acados_ocp_model();

%% acados ocp model
ocp_model.set('name', model.name);
ocp_model.set('T', T);

% symbolics
ocp_model.set('sym_x', model.x);
ocp_model.set('sym_u', model.u);
ocp_model.set('sym_xdot', model.xdot);
ocp_model.set('sym_z', model.z);
ocp_model.set('sym_p', model.p);

% dynamics
ocp_model.set('dyn_type', 'explicit');
ocp_model.set('dyn_expr_f', model.f_expl_expr);

%% State Vector Constraints
nbx = 1;
Jbx = zeros(nbx,nx);
Jbx(1,3) = 1; % Constraints on speed only
ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', SPEED_MIN);
ocp_model.set('constr_ubx', SPEED_MAX);

%% Control Vector Constraints
nbu = 2;
Jbu = zeros(nbu,nu);
Jbu(1,1) = 1; % Constraint on acceleration
Jbu(2,2) = 1; % Constraint on steering angle
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', [model.acc_min, model.delta_min]);
ocp_model.set('constr_ubu', [model.acc_max, model.delta_max]);

%% Init State Vector
state.x = ref_path(1,1);
state.y = ref_path(1,2);
state.v = 0;
state.yaw = ref_path(1,3);
initialState = [state.x, state.y, state.v, state.yaw];
model.x0 = initialState;
% set intial condition
ocp_model.set('constr_x0', model.x0);

%% Define OCP Cost Type 
% cost = define linear cost on x and u
ocp_model.set('cost_type', 'linear_ls');
ocp_model.set('cost_type_e', 'linear_ls');

%% Define the Linear Cost Expression
% number of outputs is the concatenation of x and u
ny = nx + nu + 2; % output of the running variable
ny_e = nx; % output of the terminal variable

% The linear cost contributions is defined through Vx, Vu and Vz
Vx = zeros(ny, nx);
Vx_e = zeros(ny_e, nx);
Vu = zeros(ny, nu);

Vx(1:nx,:) = eye(nx);
Vx_e(1:nx,:) = eye(nx);
Vu(nx+1:nx+2,:) = eye(nu);
Vu(nx+3:nx+4,:) = eye(nu);
ocp_model.set('cost_Vx', Vx);
ocp_model.set('cost_Vx_e', Vx_e);
ocp_model.set('cost_Vu', Vu);

% Define cost on states and input
% Matrices that produce good results according to grid search
Q = diag([ 2, 2, 20, 1]);
R = eye(nu);
R(1, 1) = 1;
R(2, 2) = 1;
Rd = eye(nu);
Rd(1,1) = 1;
Rd(2,2) = 6;
Qe = diag([ 2, 2, 20, 1]);

% Q = diag([ 2, 2, 10, 1]);
% R = eye(nu);
% R(1, 1) = 1;
% R(2, 2) = 1;
% Rd = eye(nu);
% Rd(1,1) = 1;
% Rd(2,2) = 8;
% Qe = diag([ 2, 2, 10, 1]);

% Q = diag([ 1, 1, 15, 1]);
% R = eye(nu);
% R(1, 1) = 1;
% R(2, 2) = 1;
% Rd = eye(nu);
% Rd(1,1) = 1;%1
% Rd(2,2) = 8;
% Qe = diag([ 1, 1, 15, 1]);


% Building W and W_e matrices based on Q, R, Rd and Qe
unscale = N / T;
W = unscale * blkdiag(Q, R, Rd);
W_e = Qe / unscale;
ocp_model.set('cost_W', W);
ocp_model.set('cost_W_e', W_e);

% set intial references
y_ref = zeros(ny,1);
y_ref_e = zeros(ny_e,1);
ocp_model.set('cost_y_ref', y_ref);
ocp_model.set('cost_y_ref_e', y_ref_e);

%% acados ocp set opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian); 
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('sim_method_num_stages', 4);
ocp_opts.set('sim_method_num_steps', 3);
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
ocp_opts.set('nlp_solver_tol_stat', 1e-4);
ocp_opts.set('nlp_solver_tol_eq', 1e-4);
ocp_opts.set('nlp_solver_tol_ineq', 1e-4);
ocp_opts.set('nlp_solver_tol_comp', 1e-4);

%% create ocp solver
ocp = acados_ocp(ocp_model, ocp_opts);

%% Simulate
dt = T / N;
Tf = 60.00;  % maximum simulation time [s]
Nsim = round(Tf / dt);

% initialize data structs
simX = zeros(Nsim, nx);
simU = zeros(Nsim, nu);
s0 = model.x0(1);
tcomp_sum = 0;
tcomp_max = 0;

ocp.set('constr_x0', model.x0);

% set trajectory initialization
ocp.set('init_x', model.x0' * ones(1,N+1));
ocp.set('init_u', zeros(nu, N));
ocp.set('init_pi', zeros(nx, N));

% Init Variables for MPC
pacc = 0; pdelta = 0; teta = 0;
pind = 0; tind = length(ref_path); 
goal = [ref_path(end,1) ref_path(end,2)]; finalIterationn = Nsim;

% Variables needed for Simulink
init_speed = initialState(3);
curr_pose = [initialState(1) initialState(2) initialState(4) 0];
yawrate = 0;   vx = 0;  vy = 0;   

% % Temp
% poseX = initialState(1);
% poseY = initialState(2);
% dotYaw = 0;
% Yaw = initialState(4);
% velX = 0;
% velY = 0;
% velinicial = sqrt(velX^2+velY^2);

% Logging Variables 
evalpose.x = initialState(1);   evalpose.y = initialState(2);
evalpose.v = 0;                 evalpose.z = 0;
evalpose.vx = 0;                evalpose.vy = 0;
evalpose.yaw = initialState(4); evalpose.yawrate = 0;
evalpose.poseError = 0;         evalpose.vError = 0;
evalpose.refSP = 0;             evalpose.eKin = 0;
evalpose.eKinSP = 0; 
travel = 0;

lateral_acc = zeros(Nsim, 1);

simTime(1) = 0; 
% simulate
for i = 1:Nsim
    % update reference
    [xref, dref, ind] = calc_ref_trajectory(state, ref_path, sp, pind);
    teta = ref_path(ind,6);
    pind = ind;
    
    % update running reference
    for j = 0:(N-1)
        yref = [xref(1,j+1), xref(2,j+1), xref(3,j+1), xref(4,j+1), 0, 0, pacc, pdelta];
        ocp.set('cost_y_ref', yref, j);   
    end
    
    % update terminal reference
    yref_N = [xref(1,j+2), xref(2,j+2), xref(3,j+2), xref(4,j+2)];
    ocp.set('cost_y_ref_e', yref_N);

    % solve ocp
    t = tic();
    ocp.solve();
    status = ocp.get('status'); % 0 - success
    if status ~= 0
        % borrowed from acados/utils/types.h
        %statuses = {
        %    0: 'ACADOS_SUCCESS',
        %    1: 'ACADOS_FAILURE',
        %    2: 'ACADOS_MAXITER',
        %    3: 'ACADOS_MINSTEP',
        %    4: 'ACADOS_QP_FAILURE',
        %    5: 'ACADOS_READY'
        error(sprintf('acados returned status %d in closed loop iteration %d. Exiting.', status, i));
    end
    elapsed = ocp.get('time_tot');
    %ocp.print('stat')
    %elapsed = toc(t);

    % compute elapsed and total times
    tcomp_sum = tcomp_sum + elapsed;
    if elapsed > tcomp_max
        tcomp_max = elapsed;
    end
    tcomp(i) = elapsed;

    % Retrieve state and control vector
    x0 = ocp.get('x', 0);
    u0 = ocp.get('u', 0);
    for j = 1:nx
        simX(i, j) = x0(j);
    end
    for j = 1:nu
        simU(i, j) = u0(j);
    end
    
    % save previous acceleration and steering inputs
    pacc = simU(i,1);
    pdelta = simU(i,2);
%     pdelta = 0;


%     delta = 0.0523; % 3 degrees margin
%     travel = travel + abs(x0(3)) * t_step;
%     ackangle(i) = atan(L./R_viena(pind)) + delta;
%     if R_viena(pind) < 30
%         if abs(simU(i,2)) > abs(ackangle(i))
%             if simU(i,2) > 0 
%                 understeer = true;
%                 pdelta = pdelta - 0.035;  % decrease pdelta 2 degrees when understeer occurs
%             else
%                 oversteer = true;
%                 pdelta = pdelta + 0.035;  % increase pdelta 2 degrees when oversteer occurs
%             end
%         end
%         pdelta = simU(i,2);
%     end

    
    % Simulink 
    out = sim('NEWTstRigidVehicleModelMPC','SrcWorkspace','current');
        curr_pose = out.Pose.signals.values(end,1:4);
        curr_speed = sqrt(out.Speed.signals.values(end,1)^2 + out.Speed.signals.values(end,2)^2);
        vx = out.Speed.signals.values(end,1);
        vy = out.Speed.signals.values(end,2);
        init_speed = curr_speed;
        yawrate = out.Pose.signals.values(end,4); 
        
%         poseX = out.Pose.getdatasamples(1);
%         poseY = out.Pose.getdatasamples(2);
%         Yaw = out.Pose.getdatasamples(3);
%         dotYaw = out.Pose.getdatasamples(4);
%         velX = out.Speed.getdatasamples(1);
%         velY = out.Speed.getdatasamples(2);
%         lastRow = size(poseX,1);
%         poseX = poseX(lastRow);
%         poseY = poseY(lastRow);
%         Yaw = Yaw(lastRow);
%         dotYaw = dotYaw(lastRow);
%         velX = velX(lastRow);
%         velY = velY(lastRow);
      
        disp(i)
        
    % update initial condition -> ACADOS vehicle pose
    x0 = ocp.get('x', 1);
    
    x0(1) = curr_pose(1);
    x0(2) = curr_pose(2);
    x0(3) = init_speed;
    x0(4) = curr_pose(3);
%     
%     x0(1) = out.x_dof.Data(i);
%     x0(2) = out.y_dof.Data(i);
%     x0(3) = 6.944444444444501;
%     x0(4) = out.psi_dof.Data(i);
    
    % update initial state
    ocp.set('constr_x0', x0);
    
    state.x = x0(1);
    state.y = x0(2);
    state.v = x0(3);
    state.yaw = x0(4);
    
    [poseError, vError] = computeError(state, xref);
    
    
    E_Kin = 0.5*M*(state.v)^2;
    E_Kin_SP = 0.5*M*(xref(3,1))^2;
%     
%     evalpose.x = [evalpose.x out.x_dof.Data(i)];
%     evalpose.y = [evalpose.y out.y_dof.Data(i)];
%     evalpose.yaw = [evalpose.yaw out.psi_dof.Data(i)];

    
    evalpose.x = [evalpose.x curr_pose(1)];
    evalpose.y = [evalpose.y curr_pose(2)];
    evalpose.v = [evalpose.v init_speed];
    evalpose.vx = [evalpose.vx vx];
    evalpose.vy = [evalpose.vx vy];
    evalpose.yaw = [evalpose.yaw curr_pose(3)];
    evalpose.yawrate = [evalpose.yawrate yawrate];
    evalpose.poseError = [evalpose.poseError poseError];
    evalpose.vError = [evalpose.vError vError];
    evalpose.refSP = [evalpose.refSP xref(3,1)];
    evalpose.eKin = [evalpose.eKin E_Kin];
    evalpose.eKinSP = [evalpose.eKinSP E_Kin_SP];
    
    lateral_acc(i) = simX(i, 3)^2 / R_viena(pind);


    simTime(i) = dt*i;
    
    % End if track is complete
    [goal_flag,~] = check_goal(state, goal, tind, pind);
    if goal_flag
        finalIterationn = i;
        disp('Goal Reached!')
        break
    end

end

%% Plots
t = linspace(0.0, Nsim * dt, Nsim);
zeroIdx = find(simX(:,1)==0);
if ~isempty(zeroIdx)
    simX = simX(1:zeroIdx(1),:);
    simU = simU(1:zeroIdx(1),:);
    simTime = [0 simTime];
end

figure(2)
plot(simX(1:finalIterationn,1),simX(1:finalIterationn,2), 'LineWidth', 2)
hold on
% plot(evalpose.x,evalpose.y, 'LineWidth', 2)
% hold on
% plot(evalpose1.x(1:finalIterationn),evalpose1.y(1:finalIterationn), 'LineWidth', 2)
% hold on
plot(ref_path(:,1), ref_path(:,2), 'LineWidth', 1)
grid on
legend('Vehicle Path','Objective Path')
xlabel('X [m]')
ylabel('Y [m]')


figure(3)
plot(simTime(1:finalIterationn), simX(1:finalIterationn,3), 'LineWidth',2)
hold on
% plot(t_sp1, sp1, 'LineWidth',1)
% hold on
% plot(simTime1, evalpose1.v, 'LineWidth',2)
% hold on
plot(t_sp, sp, 'LineWidth',1)
hold on
grid on
legend('Attained Speed','Speed Profile')
xlabel('Time [s]')
ylabel('Speed [m/s]')

figure(4)
plot(simTime(1:finalIterationn), lateral_acc(1:finalIterationn), 'LineWidth', 2)
grid on
xlabel('Time [s]')
ylabel('Lateral Acceleration [m/s^2]')


% Save vehicle state and control signals 
log_str = strcat('res/logforces_', path_tag, '_', pitch_tag, '.mat');
save(log_str, 'simX', 'simU', 'simTime', 'evalpose', 'ref_path', 'sp', 't_sp')
save('res/comp_time_acados.mat',tcomp)


% simX1 = simX;
% simU1 = simU;
% evalpose1 = evalpose;
% simTime1 = simTime;
% sp1 = sp;
% t_sp1 = t_sp;
% lateral_acc1 = lateral_acc;
