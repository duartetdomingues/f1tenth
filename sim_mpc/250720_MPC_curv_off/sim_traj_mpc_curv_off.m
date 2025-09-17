
% clear mex; clear classes; close all; 
clf('reset')
%close all; 
clear;
%clear mex; clear classes;

compile_solver=true;
save_traj=true;

Ts = 1/10;      % Tempo de amostragem (20 Hz)
T_s_total = 80; % Tempo de sim

% Definir o horizonte de predição
N = 20; % Horizonte do MPC

%% Parâmetros do MPC
%p= [weight_ds; weight_beta; weight_dalpha; weight_dthrottle; safety_margin];
% weight_ds = 15;
% weight_beta = 0.15;

weight_ds = 0.1;
weight_beta = 0.3;

weight_ds = 15;
weight_beta = 0.5;
weight_dalpha = 0.1;
weight_dthrottle = 0.1;
safety_margin = 0.1; % 10 cm

p=[weight_ds; weight_beta; weight_dalpha; weight_dthrottle; safety_margin];

%% Upload Traj
traj_filename="centerline_v2_test_map.csv";
%traj_filename="centerline_v2_map_2025-09-09_10-52-29.csv";
traj_dir="../../traj";

% Caminho para o ficheiro
traj_file = fullfile(traj_dir,traj_filename);

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
track.nl_traj =data(:,5);
track.nr_traj =data(:,6);

% track_width=1; % in m
% track.nl_traj =ones(length(track.kappa_traj),1)*track_width;
% track.nr_traj =ones(length(track.kappa_traj),1)*track_width;

addpath('./aux_functions/');

[left_x, left_y, right_x, right_y] = compute_track_boundaries(track);


% Plot track and boundaries
figure(1)
hold on
plot(track.x_traj, track.y_traj, 'r.', 'markersize', 10);
plot(left_x, left_y, 'b.', 'markersize', 5);
plot(right_x, right_y, 'b.', 'markersize', 5);
legend('Trajetória', 'Lado Esquerdo', 'Lado Direito');
axis equal

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
    [solver, model_var] = setup_ocp_mpc_curv(N, p ,Ts, track);  % <- Verifica se tens este ficheiro
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
% x = [0.1; 0.01; 0.01; 0.1; 0.001; 0.0001; 0.01; 0.01]; % [s; n; µ; vx; vy; r; δ; T]
% x= [0.2224;-0.0097 ;0.0314   ; 3.2755   ;-0.1606 ;   1.4739  ;  0.1993   ; 0.0996];
% x =[20.5567
%    -0.0060
%     0.0441
%     3.2940
%    -0.2095
%     1.3338
%    -0.0492
%     0.1000];
% x =[0.2224
%    -0.0060
%     0.0441
%     3.2940
%    -0.2095
%     1.3338
%    -0.0492
%     0.1000];
x =[0.1;0.1;0.1;0.1;0.1;0.0;0.0;0.0 ];

% Gráfico Trajetoria
figure(1);
clf
hold on; grid on;
plot(track.x_traj, track.y_traj, 'r--', 'LineWidth', 1.5);
plot(left_x, left_y, 'b.', 'markersize', 5);
plot(right_x, right_y, 'b.', 'markersize', 5);
%traj = plot(x(1), x(2), 'bo-', 'LineWidth', 2);
% Seta de orientação
heading_arrow = quiver(0,0, cos(0), sin(0), 0.5, 'b', 'LineWidth', 2, 'MaxHeadSize', 2);
horizon_line = plot(nan, nan, 'g.-', 'LineWidth', 1.5); % linha do horizonte    
traj_ref = plot(track.x_traj, track.y_traj, 'y--', 'LineWidth', 1.5);
ref_point = quiver(0,0, cos(0), sin(0), 0.5, 'o', 'LineWidth', 2, 'MaxHeadSize', 2);
traj_history= surface([0; 0], [0; 0], [zeros(size(0)); zeros(size(0))], ...
        [0; 0], 'facecol', 'no', 'edgecol', 'interp', 'linew', 3);
colormap(jet);
c = colorbar;
c.Label.String = 'Velocidade [m/s]';
caxis([1 4]);

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
p_d=plot(0, 0, 'r-', 'LineWidth', 2);
xlabel('Step');
ylabel('Delta [rad]');
subplot(2,1,2);
p_T=plot(0, 0, 'r-', 'LineWidth', 2);
xlabel('Step');
ylabel('Throttle');


figure(5);
sgtitle('Inputs Rate');
subplot(2,1,1);
p_u1=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('dDelta [rad/s]');
subplot(2,1,2);
p_u2=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Step');
ylabel('dThrottle [1/s]');


t_exec=0;

% Bf = 7.6671;
% Br = 7.1036;
% Cf = 1.2628;
% Cr = 1.7356;
% Df = 1.2307;
% Dr = 1.0252;

solver.set('p',p);

mpc_sim_u={};
mpc_sim_x={};

history_pred={};
history_xy=[];
history = [];
u_history=[];

% Loop principal do MPC
for t_idx = 1:T_s_total/Ts-1
    % Trajetória de referência para o horizonte atual

    solver.set('constr_lbx', x, 0);  % Estado inicial lower bound no estágio 0
    solver.set('constr_ubx', x, 0);

    tic()
    solver.solve()
    t_exec =[t_exec,toc]; 
    status =  solver.get('status');


    if status ~= 0  
        warning(['acados ocp solver failed with status ',num2str(status)]);
        solver.print('stat')

        %break
        x = x + full(model_var.f_expl_func(x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),u(1),u(2)))*Ts;
        u=u_history(end,:)';

        mpc_sim_u{t_idx} = zeros(2,10);
        mpc_sim_x{t_idx} = zeros(8,10);

        
        return
    
    else
        disp(["Sucess Solved"]);
        %solver.print('stat')

        J = solver.get('cost_value');
        V = solver.get('constraint_violation');
    
        u = solver.get('u', 0);
    
        mpc_sim_u{t_idx} = zeros(2,10);
        mpc_sim_x{t_idx} = zeros(8,10);
        for n_idx=1:N
            mpc_sim_u{t_idx}(:,n_idx)= solver.get('u',n_idx-1);
            mpc_sim_x{t_idx}(:,n_idx)= solver.get('x',n_idx);
        end

         x = solver.get('x', 1);
    end
    

     disp("phi_F = " + full(model_var.phi_F_func(x, u, p)));
     disp("phi_R = " + full(model_var.phi_R_func(x, u, p)));
    
    u_history=[u_history; u'];
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
    
    set(horizon_line, 'XData', X_h, 'YData', Y_h);
    set(ref_point,'XData', X_s, 'YData', Y_s , 'UData', cos(psi_s), 'VData', sin(psi_s));
    set(traj_history,'XData', [history_xy(:,1)'; history_xy(:,1)'], 'YData', [history_xy(:,2)';history_xy(:,2)'], 'ZData',[zeros(size(history_xy(:,1)')); zeros(size(history_xy(:,1)'))],'CData', [history(:,4)';history(:,4)']);
    set(heading_arrow,'XData', X, 'YData', Y ,'UData', cos(psi), 'VData', sin(psi));
    drawnow;

    set(p_s , 'XData', 1:t_idx, 'YData', history(:,1));
    set(p_n , 'XData', 1:t_idx, 'YData', history(:,2));
    set(p_u , 'XData', 1:t_idx, 'YData', history(:,3));
    set(p_vx , 'XData', 1:t_idx, 'YData', history(:,4));
    set(p_vy , 'XData', 1:t_idx, 'YData', history(:,5));
    set(p_r , 'XData', 1:t_idx, 'YData',history(:,6));
    set(p_d, 'XData', 1:t_idx, 'YData',history(:,7));
    set(p_T, 'XData', 1:t_idx, 'YData',history(:,8));
    set(p_u1, 'XData', 1:t_idx, 'YData',u_history(:,1));
    set(p_u2, 'XData', 1:t_idx, 'YData',u_history(:,2));
    drawnow;



end

figure(6);
plot(t_exec*1000)
title("Tempo de Execuçao Solver")
xlabel("Step")
ylabel("Tempo[ms]")
%fprintf("distancia percorrida",int(history(:,5),Ts))

figure(7);
hold on
plot(history_xy(:,1),history_xy(:,2), 'r.', 'markersize', 10);
plot(track.x_traj, track.y_traj, 'y--', 'LineWidth', 1.5);
plot(left_x, left_y, 'b.', 'markersize', 5);
plot(right_x, right_y, 'b.', 'markersize', 5);

figure(8);
clf
hold on
scatter(history_xy(:,1), history_xy(:,2), 15, history(:,4), 'filled'); % pontos coloridos
plot(left_x, left_y, 'b.', 'markersize', 5);
plot(right_x, right_y, 'b.', 'markersize', 5);
colormap(jet); % escala de cores
c = colorbar;  
c.Label.String = 'Velocidade [m/s]';   % unidades no colorbar
c.Label.FontSize = 12;
xlabel('X [m]');
ylabel('Y [m]');
title('Trajetória com velocidade como cor');
axis equal;


figure(9);
x=history_xy(:,1);
y=history_xy(:,2);
vel=history(:,4);
heading=history_xy(:,3);
hold on;
% Criar "linha 3D achatada" com cor
surface([x'; x'], [y'; y'], [zeros(size(x')); zeros(size(x'))], ...
        [vel'; vel'], 'facecol', 'no', 'edgecol', 'interp', 'linew', 3);
plot(left_x, left_y, 'b.', 'markersize', 5);
plot(right_x, right_y, 'b.', 'markersize', 5);
colorbar;
colormap(jet);
xlabel('X [m]');
ylabel('Y [m]');
title('Trajetória com velocidade como cor');
axis equal;


if false
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


    %s,x,y,kappa, nl,nr, vx
 if save_traj

    traj2 = compute_raceline_trajectory(history_xy, history);

    % Salvar a trajetória em CSV: colunas [s, x, y, kappa, vx]
    traj2_mat = [traj2.s, traj2.x, traj2.y, traj2.kappa, traj2.vx];
    traj2_dir = './traj_out';
  
    traj2_filename = extractBefore(traj_filename, ".csv") + "_raceline.csv";
    traj2_name = fullfile(traj2_dir, traj2_filename);
    % Salvar com nomes das variáveis (colunas)
    traj2_table = array2table(traj2_mat, ...
        'VariableNames', {'s', 'x', 'y', 'kappa', 'vx'});
    writetable(traj2_table, traj2_name);
    fprintf('Trajetória salva em %s\n', traj2_name);

end


















