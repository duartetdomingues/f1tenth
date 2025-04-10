%% Velocity profile
clear; close all; clc

% Load mat file
data = load("data.mat");

% Store values
progress_vel_normal = data.progress_vel_normal;
progress_vel_NN = data.progress_vel_NN;
progress_vel_pp = data.progress_vel_pp;
vel_normal = data.vel_normal;
vel_NN = data.vel_NN;
vel_pp = data.vel_pp;

% Take relevant values
progress_vel_pp = progress_vel_pp(764:1510);
vel_pp = vel_pp(764:1510);
progress_vel_normal = progress_vel_normal(593:1149);
vel_normal = vel_normal(593:1149);
progress_vel_NN = progress_vel_NN(1137:1678);
vel_NN = vel_NN(1137:1678);

% Smooth the values
% pp_spline = spline(progress_vel_pp,vel_pp);
% x_pp = linspace(progress_vel_pp(1),progress_vel_pp(end),1000);
% y_pp = fnval(pp_spline,x_pp);
% normal_spline = spline(progress_vel_normal,vel_normal);
% x_normal = linspace(progress_vel_normal(1),progress_vel_normal(end),1000);
% y_normal = fnval(normal_spline,x_normal);
% NN_spline = spline(progress_vel_NN,vel_NN);
% x_NN = linspace(progress_vel_NN(1),progress_vel_NN(end),1000);
% y_NN = fnval(NN_spline,x_NN);
x_pp = progress_vel_pp;
y_pp = vel_pp;
x_normal = progress_vel_normal;
y_normal = vel_normal;
x_NN = progress_vel_NN;
y_NN = vel_NN;


% Plot
figure('Position', [100, 100, 1200, 600]);  % Set figure size
hold on
% plot(progress_vel_pp(764:1510),vel_pp(764:1510), 'LineWidth', 1.5, 'Color', '#9bd81d')
plot(x_pp,y_pp, 'LineWidth', 1.5, 'Color', '#9bd81d')
% plot(progress_vel_normal(593:1149),vel_normal(593:1149), 'LineWidth', 1.5, 'Color', '#e42827')
plot(x_normal,y_normal, 'LineWidth', 1.5, 'Color', '#e42827')
% plot(progress_vel_NN(1137:1678),vel_NN(1137:1678),'LineWidth', 1.5, 'Color', '#1d9bd8')
plot(x_NN,y_NN,'LineWidth', 1.5, 'Color', '#1d9bd8')
legend('Pure Pursuit', 'MPCC','LMPC','Location','southeast', 'FontSize', 27)
xlim([0, 310]);
ylim([0, 20]);
ax = gca;
ax.XAxis.FontSize = 22;
ax.YAxis.FontSize = 22;
xlabel('Track Progression [m]', 'FontSize', 27)
ylabel('Velocity [m/s]', 'FontSize', 27)


% Data Info
avg_vel_pp = mean(vel_pp)   
avg_vel_normal = mean(vel_normal)
avg_vel_NN = mean(vel_NN)

lap_time_pp = 309/avg_vel_pp
lap_time_normal = 309/avg_vel_normal
lap_time_NN = 309/avg_vel_NN

%% Model Mismatch POSTER
clear; close all; clc
% Load mat file
data = load("model_mismatch.mat");

% Store values
true = data.true;
model = data.model;
NN = data.NN;

% Plot
figure('Position', [100, 100, 1200, 600]);  % Set figure size
tiledlayout(3,1)
nexttile
hold on
plot(true(:,1), 'LineWidth', 1.5, 'Color', '#9bd81d')
plot(model(:,1), 'LineWidth', 1.5, 'Color', '#e42827')
plot(NN(:,1),'LineWidth', 1.5, 'Color', '#1d9bd8')
legend('True (EKF)', 'Physical Model','Physical Model + NN','Location','northwest', 'FontSize', 20)
xlabel('Timesteps', 'FontSize', 26)
ylabel('V_x [m/s]', 'FontSize', 26)
xlim([0, 230]);
nexttile
hold on
plot(true(:,2), 'LineWidth', 1.5, 'Color', '#9bd81d')
plot(model(:,2), 'LineWidth', 1.5, 'Color', '#e42827')
plot(NN(:,2),'LineWidth', 1.5, 'Color', '#1d9bd8')
% legend('True (EKF)', 'Physical Model','Physical Model + NN','Location','Best', 'FontSize', 20)
xlabel('Timesteps', 'FontSize', 26)
ylabel('V_y [m/s]', 'FontSize', 26)
xlim([0, 230]);
nexttile
hold on
plot(true(:,3), 'LineWidth', 1.5, 'Color', '#9bd81d')
plot(model(:,3), 'LineWidth', 1.5, 'Color', '#e42827')
plot(NN(:,3),'LineWidth', 1.5, 'Color', '#1d9bd8')
% legend('True (EKF)', 'Physical Model','Physical Model + NN','Location','Best', 'FontSize', 20)
xlabel('Timesteps', 'FontSize', 26)
ylabel('Yaw Rate [rad/s]', 'FontSize', 26)
xlim([0, 230]);

%% Model Mismatch PAPER
clear; close all; clc
% Load mat file
data = load("model_mismatch.mat");

% Store values
true = data.true;
model = data.model;
NN = data.NN;

% Plot
%figure('Position', [100, 100, 1200, 600]);  % Set figure size
tiledlayout(3,1)
nexttile
hold on
plot(true(:,1), 'LineWidth', 1.5, 'Color', '#9bd81d')
plot(model(:,1), 'LineWidth', 1.5, 'Color', '#e42827')
plot(NN(:,1),'LineWidth', 1.5, 'Color', '#1d9bd8')
ax = gca;
ax.XAxis.FontSize = 22;
ax.YAxis.FontSize = 22;
% legend('True (EKF)', 'Physical Model','Physical Model + NN', 'FontSize', 18, 'Location','best')
xlabel('Timesteps', 'FontSize', 27)
ylabel('V_x [m/s]', 'FontSize', 27)
%xlim([0, 230]);
nexttile
hold on
plot(true(:,2), 'LineWidth', 1.5, 'Color', '#9bd81d')
plot(model(:,2), 'LineWidth', 1.5, 'Color', '#e42827')
plot(NN(:,2),'LineWidth', 1.5, 'Color', '#1d9bd8')
legend('True (EKF)', 'Vehicle Model','Vehicle Model + NN', 'FontSize', 27, 'Location','best')
% legend('True (EKF)', 'Physical Model','Physical Model + NN','Location','Best', 'FontSize', 20)
ax = gca;
ax.XAxis.FontSize = 22;
ax.YAxis.FontSize = 22;
xlabel('Timesteps', 'FontSize', 27)
ylabel('V_y [m/s]', 'FontSize', 27)
%xlim([0, 230]);
nexttile
hold on
plot(true(:,3), 'LineWidth', 1.5, 'Color', '#9bd81d')
plot(model(:,3), 'LineWidth', 1.5, 'Color', '#e42827')
plot(NN(:,3),'LineWidth', 1.5, 'Color', '#1d9bd8')
% legend('True (EKF)', 'Physical Model','Physical Model + NN','Location','Best', 'FontSize', 20)
ax = gca;
ax.XAxis.FontSize = 22;
ax.YAxis.FontSize = 22;
xlabel('Timesteps', 'FontSize', 27)
ylabel('Yaw Rate [rad/s]', 'FontSize', 27)
%xlim([0, 230]);