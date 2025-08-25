
%speed_to_erpm_gain  = 4277.5
%speed_to_duty (m/s) = 0.0602
% clear mex; clear classes; close all; 
close all; 
clear;
%clear mex; clear classes;

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


% Parâmetros do MPC
R = diag([1.0, 1.5]);       % Ponderação dos controles

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

if(compile_solver || ~exist('solver', 'var') )
    clear solver
    [solver, model_var] = setup_ocp_mpc_curv(N, Ts, R, track);  % <- Verifica se tens este ficheiro
    %save('solver_data.mat', 'solver');
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
x = [0.1; 0.01; 0.01; 0.1; 0.001; 0.0001; 0.01; 0.01]; % [s; n; µ; vx; vy; r; δ; T]
%x= [0.2224;-0.0097 ;0.0314   ; 3.2755   ;-0.1606 ;   1.4739  ;  0.1993   ; 0.0996];
% x =[20.5567
%    -0.0060
%     0.0441
%     3.2940
%    -0.2095
%     1.3338
%    -0.0492
%     0.1000];
x =[0.2224
   -0.0060
    0.0441
    3.2940
   -0.2095
    1.3338
   -0.0492
    0.1000];


history = x';
u0=[1;1]*0.1;
u_history=u0';

T_s_total = 20; % Tempo de sim

% Gráfico Trajetoria
figure(1);
hold on; grid on;
plot(track.x_traj, track.y_traj, 'r--', 'LineWidth', 1.5);
%traj = plot(x(1), x(2), 'bo-', 'LineWidth', 2);
% Seta de orientação
heading_arrow = quiver(0,0, cos(0), sin(0), 0.5, 'b', 'LineWidth', 2, 'MaxHeadSize', 2);
horizon_line = plot(nan, nan, 'g.-', 'LineWidth', 1.5); % linha do horizonte    
traj_ref = plot(track.x_traj, track.y_traj, 'y--', 'LineWidth', 1.5);
ref_point = quiver(0,0, cos(0), sin(0), 0.5, 'o', 'LineWidth', 2, 'MaxHeadSize', 2);
traj_history= plot(0,0, 'r.', 'markersize', 10);

xlabel('x [m]'); ylabel('y [m]');
title('Sim MPC');
legend('Trajetória de referência', 'Trajetória do veículo');
xlim([-1,10])
axis equal

figure(2);
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



figure(3);
sgtitle('States 2');
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

figure(4);
sgtitle('Inputs');
subplot(2,1,1);
p_u1=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Delta [rad]');
subplot(2,1,2);
p_u2=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('Throttle');

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

mpc_sim_u={};
mpc_sim_x={};

history_pred={};
history_xy=[];

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


    solver.set('constr_lbx', x, 0);  % Estado inicial lower bound no estágio 0
    solver.set('constr_ubx', x, 0);

    tic()
    solver.solve()
    t_exec =[t_exec,toc]; 
    status =  solver.get('status');


    if status ~= 0  
        warning(['acados ocp solver failed with status ',num2str(status)]);
        solver.print('stat')
        break
    
    else
        disp(["Sucess Solved"]);
    end
    solver.print('stat')

    u = solver.get('u', 0);

    mpc_sim_u{t_idx} = zeros(2,10);
    mpc_sim_x{t_idx} = zeros(8,10);
    for n_idx=1:N
        mpc_sim_u{t_idx}(:,n_idx)= solver.get('u',n_idx-1);
        mpc_sim_x{t_idx}(:,n_idx)= solver.get('x',n_idx);
    end

    
    u_history=[u_history; u'];

    % Atualiza estado com o modelo da bicicleta
    if status == 0  
        x = solver.get('x', 1);
    else 
        %x = x + full(model_var.f_expl_func(x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),u(1),u(2)))*Ts;
    end

    % Normalizar s
    % if x(1)>track.s_traj(end)
    %    x(1)= x(1)-track.s_traj(end);
    % end
    


    history = [history; x'];

    % Pose
    [X,Y,psi, X_s, Y_s, psi_s] = local_to_global_pose(track, model_var,x);
    history_xy = [history_xy;X,Y,psi];

    % Update Horizonte
    X_h = zeros(1, N);
    Y_h = zeros(1, N);
    for k = 1:N
        x_pred = solver.get('x', k);   % estado predito no passo k;
        if k<N
            u_pred = solver.get('u', k);   % u predito no passo k;
        end

        history_pred{t_idx}(k,:)=[x_pred; u_pred];
        [xg, yg , ~] = local_to_global_pose(track, model_var, x_pred);
        X_h(k) = xg;
        Y_h(k) = yg;
    end
    
    % if x(1)>track.s_traj(end)
    %     x(1)= mod(x(1), track.s_traj(end));
    %     solver.reset();
    %     solver.set('constr_x0', x);
    %     %solver.set('init_x', x);
    %     %solver.set('x',x)
    %     solver.set('qp_warm_start', 0);
    % end

    %set(traj, 'XData', history(:,1), 'YData', history(:,2));
    set(horizon_line, 'XData', X_h, 'YData', Y_h);
    set(ref_point,'XData', X_s, 'YData', Y_s , 'UData', cos(psi_s), 'VData', sin(psi_s));
    set(traj_history,'XData', history_xy(:,1), 'YData', history_xy(:,2));
    set(heading_arrow,'XData', X, 'YData', Y ,'UData', cos(psi), 'VData', sin(psi));


    drawnow;
    % set(traj_ref, 'XData', x_ref_step , 'YData', y_ref_step);
    % drawnow;
    set(p_s , 'XData', 0:t_idx, 'YData', history(:,1));
    set(p_n , 'XData', 0:t_idx, 'YData', history(:,2));
    set(p_u , 'XData', 0:t_idx, 'YData', history(:,3));
    set(p_vx , 'XData', 0:t_idx, 'YData', history(:,4));
    set(p_vy , 'XData', 0:t_idx, 'YData', history(:,5));
    set(p_r , 'XData', 0:t_idx, 'YData',history(:,6));
    set(p_u1, 'XData', 0:t_idx, 'YData',history(:,7));
    set(p_u2, 'XData', 0:t_idx, 'YData',history(:,8));
    drawnow;



end

figure(5);
plot(t_exec*1000)
title("Tempo de Execuçao Solver")
xlabel("Step")
ylabel("Tempo[ms]")
%fprintf("distancia percorrida",int(history(:,5),Ts))

figure(6);
subplot(2,1,1);
hold on
plot(history_xy(:,1),history_xy(:,2), 'r.', 'markersize', 10);
plot(track.x_traj, track.y_traj, 'y--', 'LineWidth', 1.5);

figure;
scatter(history_xy(:,1), history_xy(:,2), 15, history(2:end,4), 'filled'); % pontos coloridos
colormap(jet); % escala de cores
c = colorbar;  
c.Label.String = 'Velocidade [m/s]';   % unidades no colorbar
c.Label.FontSize = 12;
xlabel('X [m]');
ylabel('Y [m]');
title('Trajetória com velocidade como cor');
axis equal;


figure;
x=history_xy(:,1);
y=history_xy(:,2);
vel=history(2:end,4);
heading=history_xy(:,3);
hold on;
% Criar "linha 3D achatada" com cor
surface([x'; x'], [y'; y'], [zeros(size(x')); zeros(size(x'))], ...
        [vel'; vel'], 'facecol', 'no', 'edgecol', 'interp', 'linew', 3);

colorbar;
colormap(jet);
xlabel('X [m]');
ylabel('Y [m]');
title('Trajetória com velocidade como cor');
axis equal;

if false:
    figure('Color','w');
    hold on;
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Trajetória com velocidade como cor');
    axis equal;
    colormap(jet);
    c = colorbar;
    c.Label.String = 'Velocidade [m/s]';
    caxis([min(vel) max(vel)]);
    filename = 'trajetoria.gif';
    
    hArrow = quiver(NaN, NaN, NaN, NaN, 0, 'k', 'LineWidth', 2, ...
                    'MaxHeadSize', 2, 'AutoScale', 'off');  % 'AutoScale' off => usa dx,dy reais
    
    % Desenhar passo a passo
    for k = 2:length(x)
        % Desenha segmento até ao ponto k
        surface([x(1:k)'; x(1:k)'], [y(1:k)'; y(1:k)'], ...
                [zeros(1,k); zeros(1,k)], ...
                [vel(1:k)'; vel(1:k)'], ...
                'facecol','no','edgecol','interp','linew',3);
        % atualiza seta
        arrow_len = 0.5;                    % comprimento visual da seta
        dx = arrow_len*cos(heading(k));
        dy = arrow_len*sin(heading(k));
        set(hArrow, 'XData', x(k), 'YData', y(k), 'UData', dx, 'VData', dy);
        uistack(hArrow,'top');   
    
         drawnow;                             % atualiza render
    
        % Captura frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [A,map] = rgb2ind(im,256);
    
        % Escreve no GIF
        if k == 2
            imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.05);
        else
            imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.05);
        end
    end
 end

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

function [x,y,psi ,X_s, Y_s,theta_s]=local_to_global_pose (track, model_var,x)

    s_val=x(1);
    n_val = x(2);
    mu_val= x(3);
    
    pathlength = track.s_traj(end);
    s_val = rem(s_val, pathlength);

    X_s = interp1(track.s_traj, track.x_traj, s_val, 'linear', 'extrap');
    Y_s = interp1(track.s_traj, track.y_traj, s_val, 'linear', 'extrap');
    theta_traj = cumtrapz(track.s_traj, track.kappa_traj);  % integração de kappa para obter theta

    theta_0 = atan2(track.y_traj(2)-track.y_traj(1),track.x_traj(2)-track.x_traj(1));
    theta_traj= theta_traj+theta_0;
    theta_s = interp1(track.s_traj, theta_traj, s_val, 'linear', 'extrap');
    

    x = X_s - n_val * sin(theta_s);
    y = Y_s + n_val * cos(theta_s);
    psi = theta_s + mu_val;
end














