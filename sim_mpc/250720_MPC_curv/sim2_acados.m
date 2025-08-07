close all

compile_solver=true;

Ts = 1/10;      % Tempo de amostragem (20 Hz)
T_total = 20; % Tempo de sim
% Definir o horizonte de predição
N = 10; % Horizonte do MPC

L = 0.35; % Distância entre eixos
servo_gain=-0.840; % Ganho do servo (rad/unidade)


servo_max=0.4; % Ângulo máximo do servo
v_max = 1;        % Velocidade máxima (m/s)

delta_max = abs(servo_max/servo_gain); % Ângulo máximo de direção (radianos)

v_ref = 1.0;  % m/s

N_total = 10000;  % Numeros de pontos traj
n = (2*pi)/N_total;
x_traj = 2.5 * cos(0:n:2*pi);
y_traj = -1.75 - 1.75 * sin(0:n:2*pi);
dx = diff(x_traj);              % tamanho N-1
dy = diff(y_traj);              % tamanho N-1
psi_traj =atan2(dy, dx);           % heading entre pontos consecutivos
psi_traj(end+1) = psi_traj(end);     % repetir o último valor → agora tamanho N

figure
plot(x_traj, y_traj, 'r.', 'MarkerSize', 10);
axis equal



% Parâmetros do MPC
Q = diag([10, 10, 1, 1, 1]);  % [x, y, psi, theta, v]
R = diag([0.1, 0.1]);       % Ponderação dos controles

check_acados_requirements()
lib_path = fullfile(pwd, 'c_generated_code');

% Adiciona ao LD_LIBRARY_PATH do sistema
current_ld_path = getenv('LD_LIBRARY_PATH');
if ~contains(current_ld_path, lib_path)
    setenv('LD_LIBRARY_PATH', [getenv('LD_LIBRARY_PATH'), ':', lib_path]);
end

% Adiciona também ao path do MATLAB para segurança
addpath(lib_path);
addpath(genpath(pwd))

% ------ Setup do ACADOS ------
import casadi.*

if(compile_solver || ~exist('solver', 'var') )
    clear solver
    solver = setup_ocp(N, Ts, Q, R, delta_max, v_max);  % <- Verifica se tens este ficheiro
    save('solver_data.mat', 'solver');
else 
    % solver=acados_mex_create_mpc_model();
    % if isfile('solver_data.mat')
    %     loaded = load('solver_data.mat');
    %     solver = loaded.solver;
    % else
    %     error('Solver ainda não foi gerado! Executa primeiro com compile_solver = true');
    % end
end

% ------------------------------

% Estado inicial
x = [0; 0; 0 ;0; 0]; % [x, y, psi, theta, v]
history = x';
u0=[0;0];
u_history=u0';

% Gráfico Trajetoria
figure; hold on; grid on;
plot(x_traj, y_traj, 'r--', 'LineWidth', 1.5);
%traj = plot(x(1), x(2), 'bo-', 'LineWidth', 2);
% Seta de orientação
heading_arrow = quiver(x(1), x(2), cos(x(3)), sin(x(3)), 0.5, 'b', 'LineWidth', 2, 'MaxHeadSize', 2);
traj_ref = plot(x_traj(1:N), y_traj(1:N), 'y--', 'LineWidth', 1.5);
xlabel('x [m]'); ylabel('y [m]');
title('Sim MPC');
legend('Trajetória de referência', 'Trajetória do veículo');
xlim([-3,3])
axis equal

figure;
sgtitle('Estados');
subplot(3,1,1);
p_angle=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Ângulo de direção [rad]');
subplot(3,1,2);
p_v=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Velocidade [m/s]');
subplot(3,1,3);
p_psi=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('PSI [rad]');



figure;
sgtitle('Inputs');
subplot(2,1,1);
p_u1=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Ângulo de direção [rad]');
subplot(2,1,2);
p_u2=plot(0, 0, 'b-', 'LineWidth', 2);
ylabel('Velocidade [m/s]');
xlabel('Step');
t_exec=0;

% Loop principal do MPC
for t_idx = 1:T_total/Ts-1
    % Trajetória de referência para o horizonte atual
    

   % 1. Find the closest point to the current position
    distances = sqrt((x_traj - x(1)).^2 + (y_traj - x(2)).^2);
    [~, idx_ref_start] = min(distances);
    % 2. Compute reference points by walking along the trajectory
    x_ref_step = zeros(1, N);
    y_ref_step = zeros(1, N);
    psi_ref = zeros(1, N);
    idx = idx_ref_start;
    dist_accum = 0;

    for i = 1:N
        target_dist = (i - 1) * Ts * v_ref;
    
        while dist_accum < target_dist
            next_idx = mod(idx, length(x_traj)) + 1;
            dx = x_traj(next_idx) - x_traj(idx);
            dy = y_traj(next_idx) - y_traj(idx);
            dist_accum = dist_accum + sqrt(dx^2 + dy^2);
            idx = next_idx;
        end
    
        x_ref_step(i) = x_traj(idx);
        y_ref_step(i) = y_traj(idx);
        psi_ref(i) = psi_traj(idx);
    end

    

    % ----- Definir referências para o ACADOS -----
    % yref_all = zeros(4, N);
    % for k = 1:N
    %     yref_all(:,k) = [x_ref_step(k); y_ref_step(k); 0 ;v_ref];
    % end

    for k = 0:N-1
           yref_k = [x_ref_step(k+1); y_ref_step(k+1); sin(psi_ref(k+1));cos(psi_ref(k+1));0; v_ref];
           solver.set('cost_y_ref', yref_k, k);
    end

    %solver.set('cost_y_ref', yref_all);

    %solver.set('yref_e', [x_ref_step(end); y_ref_step(end); 0; 0; v_ref]);
    %solver.set(N, 'p', [x_ref_step(end); y_ref_step(end); v_ref]);
    
    
    solver.set('cost_y_ref_e', [x_ref_step(end); y_ref_step(end);sin(psi_ref(k+1));cos(psi_ref(k+1))]);



    solver.set('constr_lbx', x, 0);  % Estado inicial lower bound no estágio 0
    solver.set('constr_ubx', x, 0);
    
    tic()
    solver.solve();
    t_exec =[t_exec,toc]; 

    u = solver.get('u', 0);

    % U_opt = zeros(2, N);  % 2 inputs por etapa: delta e v
    % X_opt = zeros(5, N);  % 2 inputs por etapa: delta e v
    % for k = 0:N-1
    %     U_opt(:, k+1) = solver.get('u', k);
    %     X_opt(:, k+1) = solver.get('x', k);
    %     set(heading_arrow,'XData', X_opt(1,k+1), 'YData', X_opt(2,k+1),'UData', cos(X_opt(3,k+1)), 'VData', sin(X_opt(3,k+1)));
    %     drawnow;
    %     pause(0.05)
    % end
    
    u_history=[u_history; u'];

    % Atualiza estado com o modelo da bicicleta
    x = simulate_dynamics(x, u, Ts, L);

    history = [history; x'];

    % Atualiza gráfico
    %set(traj, 'XData', history(:,1), 'YData', history(:,2));
    set(heading_arrow,'XData', x(1), 'YData', x(2),'UData', cos(x(3)), 'VData', sin(x(3)));
    drawnow;
    set(traj_ref, 'XData', x_ref_step , 'YData', y_ref_step);
    drawnow;
    set(p_psi , 'XData', 0:t_idx, 'YData',history(:,3));
    set(p_angle , 'XData', 0:t_idx, 'YData', history(:,4));
    set(p_v , 'XData', 0:t_idx, 'YData',history(:,5));
    set(p_u1, 'XData', 0:t_idx, 'YData',u_history(:,1));
    set(p_u2, 'XData', 0:t_idx, 'YData',u_history(:,2));
    drawnow;



end

figure;
plot(t_exec*1000)
title("Tempo de Execuçao Solver")
xlabel("Step")
ylabel("Tempo[ms]")
%fprintf("distancia percorrida",int(history(:,5),Ts))

%Plotar angulo de direção
% figure;
% subplot(2,1,1);
% plot(1:T_total, history(:,3), 'b-', 'LineWidth', 2);
% xlabel('Step');
% ylabel('Ângulo de direção [rad]');
% title('Ângulo de direção do veículo');
% subplot(2,1,2);
% plot(1:T_total, history(:,4), 'b-', 'LineWidth', 2);
% xlabel('Step');
% ylabel('Velocidade [m/s]');
% title('Velocidade do veículo');


% ---- Simulação da Dinâmica ----
function x_sim = simulate_dynamics(x, u, Ts, L)

    delta = u(1);
    v = u(2);
    psi = x(3); % Orientação atual
    
    x(4)=delta;
    x(5)=v;
    
    x_sim = x + Ts * [
        v * cos(psi);        % dx/dt
        v * sin(psi);        % dy/dt
        (v / L) * tan(delta); % dyaw/dt
        0                     % dtheta/dt
        0                     % dv/dt = 0 (v é entrada direta)
        ];
    
    x_sim(3) = wrapToPi(x_sim(3));  % função do MATLAB

end


% ---- Função de Custo ----
function J = mpc_cost(U, x0, x_ref, y_ref, v_ref, Ts, L, N, Q, R)
    x = x0;
    J = 0;
    
    for k = 1:N
        u = U(k, :)'; % Controle atual
    
        x = simulate_dynamics(x, u, Ts, L);
        
    
        % Erro em relação à referência
        e = [x(1)-x_ref(k); x(2)-y_ref(k); 0; 0 ;x(5)-v_ref];
        J = J + e' * Q * e + u' * R * u;
    end
end