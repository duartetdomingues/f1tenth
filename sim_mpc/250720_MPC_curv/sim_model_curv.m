
%speed_to_erpm_gain  = 4277.5
%speed_to_duty (m/s) = 0.0602
close all
%


compile_solver=true;

Ts = 1/10;      % Tempo de amostragem (20 Hz)
% Definir o horizonte de predição
N = 10; % Horizonte do MPC

L = 0.35; % Distância entre eixos
servo_gain=-0.840; % Ganho do servo (rad/unidade)

servo_max_rate = 5.2360/10;


servo_max=0.4; % Ângulo máximo do servo
v_max = 2.5;        % Velocidade máxima (m/s)

delta_max = abs(servo_max/servo_gain); % Ângulo máximo de direção (radianos)

%% Upload Traj
traj_file="centerline_map_2025-07-16_15-28-18.csv";
traj_dir="../../traj";

% Caminho para o ficheiro
traj_file = fullfile(traj_dir,traj_file);

% Verificar se o ficheiro existe
if ~isfile(traj_file)
    error('Ficheiro não encontrado: %s', traj_file);
end

% Carregar o CSV
data = readmatrix(traj_file);  % ou: csvread(traj_file); para versões mais antigas

% Verificar conteúdo
track.s_traj=data(:,1);
track.x_traj=data(:,2);
track.y_traj=data(:,3);
track.kappa_traj=data(:,4);
track.nl_traj =ones(length(track.kappa_traj),1);
track.nr_traj =ones(length(track.kappa_traj),1);

 % figure
 % plot(x_traj, y_traj, 'r.', 'markersize', 10);
 % axis equa




check_acados_requirements()
lib_path = fullfile(pwd, 'c_generated_code/');

% Adiciona ao LD_LIBRARY_PATH do sistema
current_ld_path = getenv('LD_LIBRARY_PATH');
disp("Current_ld_path:");
disp(current_ld_path)
if ~contains(current_ld_path, lib_path)
    disp("Add c_generated_code Path to LD_LIBRARY_PATH:")
    disp(lib_path)
    setenv('LD_LIBRARY_PATH', [getenv('LD_LIBRARY_PATH'), ':', lib_path]);
    current_ld_path = getenv('LD_LIBRARY_PATH');
    disp("Next_ld_path:");
    disp(current_ld_path)
end

% Adiciona também ao path do MATLAB para segurança
addpath(lib_path);
addpath(genpath(pwd))

% ------ Setup do ACADOS ------
import casadi.*


% ------------------------------

% Estado inicial
x = [0;0;0;0;0;0;0;0]; % [s; n; µ; vx; vy; r; δ; T]
history = x';
history_xy=[0,0];
u0=[-1;1]*0.1;
u_history=u0';

T_s_total = 5; % Tempo de sim

% Gráfico Trajetoria
figure; hold on; grid on;
plot(track.x_traj, track.y_traj, 'r--', 'LineWidth', 1.5);
%traj = plot(x(1), x(2), 'bo-', 'LineWidth', 2);
% Seta de orientação
heading_arrow = quiver(0,0, cos(0), sin(0), 0.5, 'b', 'LineWidth', 2, 'MaxHeadSize', 2);
traj_xy = plot(history_xy(:,1),history_xy(:,2), 'r.', 'markersize', 10);
horizon_line = plot(nan, nan, 'g.-', 'LineWidth', 1.5); % linha do horizonte    
traj_ref = plot(track.x_traj, track.y_traj, 'y--', 'LineWidth', 1.5);
ref_point = quiver(0,0, cos(0), sin(0), 0.5, 'o', 'LineWidth', 2, 'MaxHeadSize', 2);

xlabel('x [m]'); ylabel('y [m]');
title('Sim MPC');
legend('Trajetória de referência', 'Trajetória do veículo');
axis equal

figure;
sgtitle('Estados');
subplot(3,1,1);
p_s=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Progresso [m]');
subplot(3,1,2);
p_n=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Desvio lateral  [m]');
subplot(3,1,3);
p_u=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Heading relativo [rad]');



figure;
sgtitle('Inputs');
subplot(3,1,1);
p_vx=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Velocidade X [m/s]');
subplot(3,1,2);
p_vy=plot(0, 0, 'b-', 'LineWidth', 2);
ylabel('Velocidade Y [m/s]');
xlabel('Step');
subplot(3,1,3);
p_r=plot(0, 0, 'b-', 'LineWidth', 2);
ylabel('Yaw rate [rad/s]');
xlabel('Step');

figure;
sgtitle('Inputs/States');
subplot(2,2,1);
p_u1=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Delta U [rad]');
subplot(2,2,2);
p_u2=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Throttle U');
subplot(2,2,3);
p_d=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Delta X[rad]');
subplot(2,2,4);
p_t=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Throttle X');

t_exec=0;


Bf = 7.6671;
Br = 7.1036;
Cf = 1.2628;
Cr = 1.7356;
Df = 1.2307;
Dr = 1.0252;

Bf = 0.1;
Br = 0.1;
Cf = 1;
Cr = 1;
Df = 1;
Dr = 1;

p_global=[true,Bf,Br,Cf,Cr,Df,Dr]';

%p_global=zeros(6,1);

%solver.set_p_global_and_precompute_dependencies(p_global)

%solver.set("p_global",p_global')

%solver.set('p',[Ts; servo_max_rate]);

mpc_sim_u=[];
mpc_sim_x=[];


history_pred={};

logs = struct('Fy_f',[],'Fy_r',[],'alfa_f',[],'alfa_r',[]); 

last_s=0;

% Loop principal do MPC
for t_idx = 1:T_s_total/Ts-1
    % Trajetória de referência para o horizonte atual
    

   % 1. Update the kappa(s)

    

    % ----- Definir referências para o ACADOS -----
    % yref_all = zeros(4, N);
    % for k = 1:N
    %     yref_all(:,k) = [x_ref_step(k); y_ref_step(k); 0 ;v_ref];
    % end

    %solver.set('cost_y_ref', yref_all);

    %solver.set('yref_e', [x_ref_step(end); y_ref_step(end); 0; 0; v_ref]);
    %solver.set(N, 'p', [x_ref_step(end); y_ref_step(end); v_ref]);


    
    


    u = u0;

    % 
    % mpc_sim_u{t_idx}= solver.get('u');
    % mpc_sim_x{t_idx}= solver.get('x');

    
    u_history=[u_history; u'];

    % Atualiza estado com o modelo da bicicleta
    [x, log_step] = car_model_curv(Ts, track,x,u);

    logs.Fy_f=[logs.Fy_f,log_step.Fy_f];
    logs.Fy_r=[logs.Fy_r,log_step.Fy_r];
    logs.alfa_f=[logs.alfa_f,log_step.alfa_f];
    logs.alfa_r=[logs.alfa_r,log_step.alfa_r];

    x=full(x);

    last_s = history(end,1);

    history = [history; x'];

    %x(1)=last_s;

    % Pose
    [X,Y,psi, X_s, Y_s, psi_s] = local_to_global_pose(track,x, last_s);
    history_xy = [history_xy;X,Y];

    % Update Horizonte
    % X_h = zeros(1, N);
    % Y_h = zeros(1, N);
    % for k = 1:N
    %     x_pred = solver.get('x', k);   % estado predito no passo k;
    %     if k<N
    %         u_pred = solver.get('u', k);   % u predito no passo k;
    %     end
    % 
    %     history_pred{t_idx}(k,:)=[x_pred; u_pred];
    %     [xg, yg, ~] = local_to_global_pose(track, x_pred);
    %     X_h(k) = xg;
    %     Y_h(k) = yg;
    % end

    %set(traj, 'XData', history(:,1), 'YData', history(:,2));
    set(heading_arrow,'XData', X, 'YData', Y ,'UData', cos(psi), 'VData', sin(psi));
    set(ref_point,'XData', X_s, 'YData', Y_s , 'UData', cos(psi_s), 'VData', sin(psi_s));


    set(traj_xy,'XData', history_xy(:,1),'YData', history_xy(:,2));
    % set(horizon_line, 'XData', X_h, 'YData', Y_h);

    drawnow;
    % set(traj_ref, 'XData', x_ref_step , 'YData', y_ref_step);
    % drawnow;
    set(p_s , 'XData', 0:t_idx, 'YData', history(:,1));
    set(p_n , 'XData', 0:t_idx, 'YData', history(:,2));
    set(p_u , 'XData', 0:t_idx, 'YData', history(:,3));
    set(p_vx , 'XData', 0:t_idx, 'YData', history(:,4));
    set(p_vy , 'XData', 0:t_idx, 'YData', history(:,5));
    set(p_r , 'XData', 0:t_idx, 'YData',history(:,6));
    set(p_d, 'XData', 0:t_idx, 'YData',history(:,7));
    set(p_t, 'XData', 0:t_idx, 'YData',history(:,8));
    set(p_u1, 'XData', 0:t_idx, 'YData',u_history(:,1));
    set(p_u2, 'XData', 0:t_idx, 'YData',u_history(:,2));
    drawnow;


    if t_idx==32
        disp("stop")
    end



end

figure;
plot(t_exec*1000)
title("Tempo de Execuçao Solver")
xlabel("Step")
ylabel("Tempo[ms]")
%fprintf("distancia percorrida",int(history(:,5),Ts))

figure;
subplot(2,1,1);
hold on
plot(history_xy(:,1),history_xy(:,2), 'r.', 'markersize', 10);
plot(track.x_traj, track.y_traj, 'y--', 'LineWidth', 1.5);
subplot(2,1,2);
plot(history(:,2))

% %Plotar angulo de direção
% figure;
% subplot(2,1,1);
% plot(history(:,4), 'b-', 'LineWidth', 2);
% xlabel('Step');
% ylabel('Ângulo de direção [rad]');
% title('Ângulo de direção do veículo');
% subplot(2,1,2);
% plot(diff(history(:,4))/Ts, 'b-', 'LineWidth', 2);
% xlabel('Step');
% ylabel('Ângulo de direção derivado [rad]');
% title('Derivada do Ângulo de direção do veículo');
% 
% 
% x_delta = mpc_sim_x{1}(4,2:end);
% u_delta = mpc_sim_u{1}(1,:);
% error = x_delta - u_delta;
% disp('Error u1 vs x4:')
% disp(error)
% 
% 
% max_diff_u=0;
% for i =1:length(mpc_sim_u)
%     diff_u = abs(mpc_sim_u{i}(1,:)-mpc_sim_x{i}(4,1:end-1))/Ts;
%     if max_diff_u<max(diff_u)
%         max_diff_u=max(diff_u);
%     end
% end
% 
% disp(max_diff_u)

function [x,y,psi, X_s, Y_s,theta_s]=local_to_global_pose (track,x,s)

    s_val = s;
    n_val = x(2);
    mu_val = x(3);
    
    import casadi.*

    x_lut =interpolant('kappa_lut','linear',{track.s_traj},track.x_traj);
    y_lut =interpolant('n_l_lut','linear',{track.s_traj},track.y_traj);
    theta_traj = cumsum(track.kappa_traj * 0.01);
    theta_0 = atan2(track.y_traj(2)-track.y_traj(1),track.x_traj(2)-track.x_traj(1));
    %theta_traj= theta_traj+theta_0;
    % dx = cos(theta_traj);  % Derivada de x em relação a theta
    % dy = sin(theta_traj); 
    
    theta_lut =interpolant('n_r_lut','linear',{track.s_traj},theta_traj);

   


    X_s = full(x_lut(s_val));
    Y_s = full(y_lut(s_val));
    theta_s = full(theta_lut(s_val));
    
    x = full(X_s + n_val * cos(theta_s+3.1416/2));
    y = full(Y_s + n_val * sin(theta_s+3.1416/2));
    psi = full(theta_s + mu_val);
end














