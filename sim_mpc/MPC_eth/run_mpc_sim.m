% run_mpc_sim.m
% Script para montar o OCP (acados_settings + bicycle_model) e simular o MPC.
% Requer:
%   - bicycle_model.m
%   - acados_settings.m
%   - aux_functions/ (compute_track_boundaries, local_to_global_pose, compute_raceline_trajectory)
%
% Dica: ajuste stmpc_config, car_config e tire_config conforme o seu carro/mapa.

clf('reset'); clear; clc;
%close all;

%% -------------------- FLAGS E TEMPO ---------------------------------------
compile_solver = true;
save_traj      = true;

Ts         = 1/20;   % 10 Hz
T_s_total  = 30;     % tempo total de simulação [s]
stmpc_N    = 40;     % horizonte do MPC

%% -------------------- LEITURA DA PISTA ------------------------------------
%traj_filename = "centerline_test_map.csv";
traj_filename="track_data.csv";

traj_dir      = "../../traj";
traj_file     = fullfile(traj_dir, traj_filename);

if ~isfile(traj_file)
    error('Ficheiro não encontrado: %s', traj_file);
end

data = readmatrix(traj_file);
track.s_traj     = data(:,1);
track.x_traj     = data(:,2);
track.y_traj     = data(:,3);
track.kappa_traj = data(:,4);
track.nl_traj    = data(:,5);
track.nr_traj    = data(:,6);

addpath('./aux_functions/');
[left_x, left_y, right_x, right_y] = compute_track_boundaries(track);

figure(1); hold on; grid on; axis equal;
plot(track.x_traj, track.y_traj, 'r.', 'markersize', 10);
plot(left_x,  left_y,  'b.', 'markersize', 5);
plot(right_x, right_y, 'b.', 'markersize', 5);
legend('Trajetória', 'Lado Esquerdo', 'Lado Direito');
title('Mapa e limites');

%% -------------------- ACADOS PATHS ----------------------------------------
if exist('check_acados_requirements','file')
    check_acados_requirements();
end
lib_path = fullfile(pwd, 'c_generated_code/');
current_ld_path = getenv('LD_LIBRARY_PATH');
if ~contains(string(current_ld_path), string(lib_path))
    setenv('LD_LIBRARY_PATH', [getenv('LD_LIBRARY_PATH'), ':', lib_path]);
end
addpath(lib_path);
addpath(genpath(pwd));

%% -------------------- CONFIGURAÇÕES ---------------------------------------
% Frequência do MPC (coerente com Ts)
stmpc_config = struct();

% (Opcional) atrasos — guardo para referência, só terão efeito se modelares no OCP
stmpc_config.t_delay     = 0.0025;   % [s]
stmpc_config.steps_delay = 3;

% Horizonte e frequência
stmpc_config.N        = 40;      % YAML: N: 40
stmpc_config.MPC_freq = 1/Ts;    % manter coerência com Ts (define Ts = 1/20 p/ bater certo)

% Margens e geometria
stmpc_config.track_safety_margin = 0.25;    % [m]
stmpc_config.track_max_width     = 1e3;     % [m]
stmpc_config.overtake_d          = 1.0;     % [m]

% Pesos (custo)
stmpc_config.qjerk   = 1e-2;
stmpc_config.qddelta = 5e-1;
stmpc_config.qadv    = 0;
stmpc_config.qn      = 40;
stmpc_config.qalpha  = 0.1;
stmpc_config.qv      = 10;

% Slacks
stmpc_config.Zl = 1000; stmpc_config.Zu = 1000;
stmpc_config.zl = 1000; stmpc_config.zu = 1000;

% Limites de estado
stmpc_config.delta_min  = -0.4;   stmpc_config.delta_max  =  0.4;
stmpc_config.v_min      =  0.0;   stmpc_config.v_max      = 12.0;
stmpc_config.a_min      = -10.0;  stmpc_config.a_max      = 10.0;

% Limites de input
stmpc_config.ddelta_min = -3.2;   stmpc_config.ddelta_max =  3.2;
stmpc_config.jerk_min   = -50.0;  stmpc_config.jerk_max   = 50.0;

% Constraint não linear
stmpc_config.alat_max = 10.0;     % [m/s^2]

% Flags de custo/constraints/modelo
stmpc_config.vy_minimization      = true;
stmpc_config.adv_maximization     = false;
stmpc_config.combined_constraints = "None";   % 'ellipse' / 'diamond' / 'None'
stmpc_config.load_transfer        = true;
stmpc_config.correct_v_y_dot      = true;

stmpc_config.Zl = 1000; stmpc_config.Zu = 1000;
stmpc_config.zl = 1000; stmpc_config.zu = 1000;

% Outros toggles
stmpc_config.vy_minimization   = true;
stmpc_config.adv_maximization  = false;
stmpc_config.combined_constraints = ''; % 'ellipse' ou 'diamond' se quiser usar
stmpc_config.load_transfer     = true;

% largura "global" usada em constraints agregadas (apenas para bounds simbólicos extra)
stmpc_config.track_max_width   = 1000;

% Veículo (exemplo tipo RC)
car_config = struct();
car_config.lf   = 0.13;
car_config.lr   = 0.13;
car_config.m    = 3.0;     % [kg]
car_config.Iz   = 0.03;    % [kg m^2]
car_config.h_cg = 0.05;    % [m]

% Pneus (parâmetros simples tipo Pacejka reduzidos)
tire_config = struct();
tire_config.friction_coeff = 1.0;
tire_config.Bf = 7.6671;   tire_config.Br = 7.1036;
tire_config.Cf = 1.2628;   tire_config.Cr = 1.7356;
tire_config.Df = 1.2307;   tire_config.Dr = 1.0252;
tire_config.Ef = 0.0;      tire_config.Er = 0.0;

%% -------------------- CRIAÇÃO DO SOLVER -----------------------------------
import casadi.*

s0        = track.s_traj(:).';
kapparef  = track.kappa_traj(:).';
d_left    = track.nl_traj(:).';
d_right   = track.nr_traj(:).';

[constraint, model_var, solver, params] = acados_settings(...
    s0, kapparef, d_left, d_right, stmpc_config, car_config, tire_config);

% Garantir que os parâmetros passaram (pode voltar a definir online)
p_vec = [ ...
    2;                                 % V_target placeholder (é ignorado aqui)
    stmpc_config.qadv;                 % weight_adv
    stmpc_config.qv;                   % weight_qv
    stmpc_config.qn;                   % weight_qn
    stmpc_config.qalpha;               % weight_qalpha
    stmpc_config.qjerk;                % weight_qjerk
    stmpc_config.qddelta;              % weight_ddelta
    2.5;                               % a_lat_max [m/s^2] (ajuste)
    stmpc_config.a_min;                % a_long_min
    stmpc_config.a_max;                % a_long_max
    stmpc_config.track_safety_margin;  % safety_margin
    stmpc_config.overtake_d            % overtake_d
    ];
solver.set('p', p_vec);

%% -------------------- ESTADO INICIAL --------------------------------------
% x = [s; n; theta; v_x; v_y; delta; yaw_rate; accel]
x = [27.5; 0.01; 0.01; 0.01; 0.0; 0.0; 0.0; 0.0];

% Preparar figuras
figure(1); clf; hold on; grid on;
plot(track.x_traj, track.y_traj, 'r--', 'LineWidth', 1.0);
plot(left_x,  left_y,  'b.', 'markersize', 5);
plot(right_x, right_y, 'b.', 'markersize', 5);
heading_arrow = quiver(0,0, cos(0), sin(0), 0.5, 'b', 'LineWidth', 2, 'MaxHeadSize', 2);
horizon_line  = plot(nan, nan, 'g.-', 'LineWidth', 1.5);
traj_history  = surface([0; 0], [0; 0], [0 0; 0 0], [0; 0], 'facecol', 'no', 'edgecol', 'interp', 'linew', 3);
colormap(jet); c = colorbar; c.Label.String = 'Velocidade [m/s]'; caxis([0 4]);
xlabel('x [m]'); ylabel('y [m]'); title('Sim MPC'); axis equal;

figure(2); clf; sgtitle('Estados');
subplot(3,1,1); p_s = plot(0,0,'b-'); ylabel('s [m]');
subplot(3,1,2); p_n = plot(0,0,'b-'); ylabel('n [m]');
subplot(3,1,3); p_u = plot(0,0,'b-'); ylabel('\theta [rad]'); xlabel('step');

figure(3); clf; sgtitle('States 2');
subplot(3,1,1); p_vx = plot(0,0,'b-'); ylabel('v_x [m/s]');
subplot(3,1,2); p_vy = plot(0,0,'b-'); ylabel('v_y [m/s]'); xlabel('step');
subplot(3,1,3); p_r  = plot(0,0,'b-'); ylabel('yaw rate [rad/s]'); xlabel('step');

figure(4); clf; sgtitle('Inputs');
subplot(2,1,1); p_d = plot(0,0,'r-'); ylabel('\delta [rad]');
subplot(2,1,2); p_T = plot(0,0,'r-'); ylabel('a [m/s^2]'); xlabel('step');

figure(5); clf; sgtitle('Inputs Rate');
subplot(2,1,1); p_u1 = plot(0,0,'b-'); ylabel('d\delta [rad/s]');
subplot(2,1,2); p_u2 = plot(0,0,'b-'); ylabel('jerk [m/s^3]'); xlabel('step');

%% -------------------- SIMULAÇÃO MPC ---------------------------------------
t_exec = [];
u_history = [];
history   = [];
history_xy = [];
history_pred_x = {};
history_pred_u = {};

sim_steps = floor(T_s_total / Ts);

for t_idx = 1:sim_steps
    % Condição inicial (stage 0)
    solver.set('x', x, 0);
    solver.set('constr_x0', x);

    % (Opcional) Warm-start
    % for k = 1:stmpc_N
    %     solver.set('x', x, k);
    % end

    tic;
    solver.solve();
    t_exec(end+1,1) = toc;

    solver_status =  solver.get('status');

    if solver_status ~= 0
        warning(['acados ocp solver failed with status ', num2str(solver_status)]);
        solver.print('stat');
        break; % aborta sim para debug
    end

    % Inputs e estados previstos
    u0 = solver.get('u', 0);
    % Guardar previsões (opcional)
    history_pred_u{t_idx} = zeros(2,stmpc_N-1);
    history_pred_x{t_idx} = zeros(8,stmpc_N);
    for k = 1:stmpc_N
        xk = solver.get('x', k);
        history_pred_x{t_idx}(:,k) = xk;
        if k <= stmpc_N-1
            history_pred_u{t_idx}(:,k) = solver.get('u', k);
        end
    end

    % Avançar para o próximo estado (pega x(1) do horizonte)
    x = solver.get('x', 1);

    % Logs
    u_history = [u_history; u0'];
    history   = [history;   x'];

    % Pose global (requer aux_functions/local_to_global_pose)
    try
        [X, Y, psi, Xs, Ys, psis] = local_to_global_pose(track, model_var, x);
        history_xy = [history_xy; X, Y, psi];
    catch
        % fallback simples: aprox posição pela trajetória nominal (sem rotação correta)
        s_idx = max(1, min(length(track.s_traj), round(x(1))));
        X = track.x_traj(s_idx); Y = track.y_traj(s_idx); psi = 0;
        Xs = X; Ys = Y; psis = psi;
        history_xy = [history_xy; X, Y, psi];
    end

    % Actualizar gráficos
    X_h = nan(1, stmpc_N); Y_h = nan(1, stmpc_N);
    for k = 1:stmpc_N
        xk = solver.get('x', k);
        try
            [xg, yg, ~] = local_to_global_pose(track, model_var, xk);
        catch
            s_idx = max(1, min(length(track.s_traj), round(xk(1))));
            xg = track.x_traj(s_idx); yg = track.y_traj(s_idx);
        end
        X_h(k) = xg; Y_h(k) = yg;
    end

    set(horizon_line, 'XData', X_h, 'YData', Y_h);
    set(traj_history,'XData', [history_xy(:,1)'; history_xy(:,1)'], ...
                     'YData', [history_xy(:,2)'; history_xy(:,2)'], ...
                     'ZData', [zeros(1,size(history_xy,1)); zeros(1,size(history_xy,1))], ...
                     'CData', [history(:,4)'; history(:,4)']);
    set(heading_arrow,'XData', X, 'YData', Y, 'UData', cos(psi), 'VData', sin(psi));
    drawnow;

    set(p_s,  'XData', 1:t_idx, 'YData', history(:,1));
    set(p_n,  'XData', 1:t_idx, 'YData', history(:,2));
    set(p_u,  'XData', 1:t_idx, 'YData', history(:,3));
    set(p_vx, 'XData', 1:t_idx, 'YData', history(:,4));
    set(p_vy, 'XData', 1:t_idx, 'YData', history(:,5));
    set(p_r,  'XData', 1:t_idx, 'YData', history(:,7));
    set(p_d,  'XData', 1:t_idx, 'YData', history(:,6));
    set(p_T,  'XData', 1:t_idx, 'YData', history(:,8));
    set(p_u1, 'XData', 1:t_idx, 'YData', u_history(:,2)); % derDelta
    set(p_u2, 'XData', 1:t_idx, 'YData', u_history(:,1)); % jerk

end

%% -------------------- FIGURAS FINAIS --------------------------------------
figure(6); clf; plot(t_exec*1000, 'LineWidth',1.5);
title("Tempo de Execução do Solver"); xlabel("Step"); ylabel("Tempo [ms]"); grid on;

figure(7); hold on; grid on; axis equal;
plot(history_xy(:,1), history_xy(:,2), 'r.', 'markersize', 10);
plot(track.x_traj, track.y_traj, 'y--', 'LineWidth', 1.0);
plot(left_x,  left_y,  'b.', 'markersize', 5);
plot(right_x, right_y, 'b.', 'markersize', 5);
title('Trajetória');

figure(8); clf; hold on; axis equal; grid on;
scatter(history_xy(:,1), history_xy(:,2), 15, history(:,4), 'filled');
plot(left_x,  left_y,  'b.', 'markersize', 5);
plot(right_x, right_y, 'b.', 'markersize', 5);
colormap(jet); c = colorbar; c.Label.String = 'Velocidade [m/s]';
xlabel('X [m]'); ylabel('Y [m]'); title('Trajetória com velocidade como cor');

figure(9); clf; hold on; axis equal; grid on;
xv=history_xy(:,1); yv=history_xy(:,2); vel=history(:,4);
surface([xv'; xv'], [yv'; yv'], [zeros(size(xv')); zeros(size(xv'))], ...
        [vel'; vel'], 'facecol','no','edgecol','interp','linew',3);
plot(left_x,  left_y,  'b.', 'markersize', 5);
plot(right_x, right_y, 'b.', 'markersize', 5);
colorbar; colormap(jet);
xlabel('X [m]'); ylabel('Y [m]'); title('Trajetória com velocidade como cor');

%% -------------------- EXPORT (OPCIONAL) -----------------------------------
if save_traj && exist('compute_raceline_trajectory','file')
    traj2 = compute_raceline_trajectory(history_xy, history);
    traj2_mat = [traj2.s, traj2.x, traj2.y, traj2.kappa, traj2.vx];
    traj2_dir = './traj_out';
    if ~exist(traj2_dir, 'dir'); mkdir(traj2_dir); end
    traj2_filename = extractBefore(traj_filename, ".csv") + "_raceline.csv";
    traj2_name = fullfile(traj2_dir, traj2_filename);
    traj2_table = array2table(traj2_mat, 'VariableNames', {'s','x','y','kappa','vx'});
    writetable(traj2_table, traj2_name);
    fprintf('Trajetória salva em %s\n', traj2_name);
end
