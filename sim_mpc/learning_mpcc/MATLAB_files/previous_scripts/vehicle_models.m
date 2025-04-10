close all
% Choose and read config file
addpath("yaml_parser/")
config_data =  yaml.loadFile('/home/david/fst/autonomous-systems/src/control/learning_mpcc/config/default.yaml');
% Define Model parameters
global car

car.l_f = config_data.model_params.l_f;
car.l_r = config_data.model_params.l_r;
car.m = config_data.model_params.m;
car.I_z = config_data.model_params.I_z;
car.T_max = config_data.model_params.T_max;
car.T_brake = config_data.model_params.T_brake;
car.GR = config_data.model_params.GR;
car.eta_motor = config_data.model_params.eta_motor;
car.r_wheel = config_data.model_params.r_wheel;
car.g = config_data.model_params.g;
car.C_roll = config_data.model_params.C_roll;
car.rho = config_data.model_params.rho;
car.lambda_blend_min = config_data.model_params.lambda_blend_min;
car.lambda_blend_max = config_data.model_params.lambda_blend_max;
car.C_d = config_data.model_params.C_d;
car.C_l = config_data.model_params.C_l;
car.downforce_front = config_data.model_params.C_d;
car.downforce_rear = config_data.model_params.C_l;
car.lambda_blend_min = config_data.model_params.lambda_blend_min;
car.lambda_blend_max = config_data.model_params.lambda_blend_max;
% Define Tyre parameters
car.B = config_data.tyre_params.B;
car.C = config_data.tyre_params.C;
car.D = config_data.tyre_params.D;
% car.E = config_data.tyre_params.E;

%% Dynamic Model

size = 1000;
history = zeros(8,size);

x = [0 0 0 0 0 0]'; %xinit
u = [0.2, 0.1]';

for i = 1:size
    [dx,alpha_f,alpha_r] = dynamic_bicycle_model_func_new(x,u);
    x = x + 0.01*dx;
    history(:,i) = [x; alpha_f; alpha_r];
end

figure(1)
hold on
tiledlayout(2,4)
nexttile
plot(history(1,:),history(2,:));
daspect([1 1 1])
title("X and Y")
nexttile
plot(history(3,:));
title("Psi");
nexttile
plot(history(4,:))
title("vx")
nexttile
plot(history(5,:))
title("vy")
nexttile
plot(history(6,:))
title("r")
nexttile
plot(history(7,:))
title("\alpha_f")
nexttile
plot(history(8,:))
title("\alpha_r")
sgtitle('Dynamic')
hold off;


%% Kinematic Model

size = 1000;
history_kin = zeros(6,size);

x = [0 0 0 0 0 0]'; %xinit
u = [0.2, 0.1]';

for i = 1:size
    dx = kinematic_bicycle_model_func(x,u);
    x = x + 0.01*dx;
    history_kin(:,i) = x;
end

figure(2)
hold on
tiledlayout(3,2)
nexttile
plot(history_kin(1,:),history_kin(2,:));
daspect([1 1 1])
title("X and Y")
nexttile
plot(history_kin(3,:));
title("Psi");
nexttile
plot(history_kin(4,:))
title("vx")
nexttile
plot(history_kin(5,:))
title("vy")
nexttile
plot(history_kin(6,:))
title("r")
nexttile
hold off
sgtitle('Kinematic')

%% Blended Model

size = 1000;
history_blended = zeros(6,size);

x = [0 0 0 0 0 0]'; %xinit
u = [0.2, 0.1]';

for i = 1:size
    dx_kin = kinematic_bicycle_model_func(x,u);
    [dx_dyn,~,~] = dynamic_bicycle_model_func_new(x,u);
    x_kin = x + 0.01*dx_kin;
    x_dyn = x + 0.01*dx_dyn;
    x = blended_func(x_kin,x_dyn);
    history_blended(:,i) = x;
end

figure(3)
hold on
tiledlayout(3,2)
nexttile
plot(history_blended(1,:),history_blended(2,:));
daspect([1 1 1])
title("X and Y")
nexttile
plot(history_blended(3,:));
title("Psi");
nexttile
plot(history_blended(4,:))
title("vx")
nexttile
plot(history_blended(5,:))
title("vy")
nexttile
plot(history_blended(6,:))
title("r")
hold off
sgtitle('Blended')

%% Plot Tyre model

figure(4)

x = linspace(-1,1);

Ff_y = 2*car.D*sin(car.C*atan(car.B.*x));

plot(x,Ff_y)
title('Lateral Force')


%% Dynamic Bicycle Model Old

function [dX, alpha_f] = dynamic_bicycle_model_func(x,u)
    global car

    % x = [1 - X
    %      2 - Y
    %      3 - Psi
    %      4 - vx
    %      5 - vy
    %      6 - r]

    % u = [1 - d 
    %      2 - delta]

    alpha_f = u(2) - atan2(x(5) + x(6)*car.l_f, x(4));
    alpha_r = -atan2(x(5) - x(6)*car.l_r, x(4));
%     Ff_y = -2*car.D*sin(car.C*atan(car.B*alpha_f-car.E*(car.B*alpha_f-atan(car.B*alpha_f))));
%     Fr_y = -2*car.D*sin(car.C*atan(car.B*alpha_r-car.E*(car.B*alpha_r-atan(car.B*alpha_r))));
    Ff_y = -2*car.D*sin(car.C*atan(car.B*alpha_f));
    Fr_y = -2*car.D*sin(car.C*atan(car.B*alpha_r));
    
    t_bool = (u(1) < 0);
    T = car.T_max + t_bool*(car.T_brake - car.T_max);
    F_prop = 2*car.eta_motor*T*car.GR/car.r_wheel*u(1);
    
    F_roll = car.C_roll*car.m*car.g;
    
    F_drag = 1/2*car.rho*car.C_d*x(4)^2;
    
    cos_delta = cos(u(2));
    F_x = F_prop - F_roll - F_drag - Ff_y*sin(u(2));
    F_y = Ff_y*cos_delta + Fr_y;
    M_z = Ff_y*cos_delta*car.l_f - Fr_y*car.l_r;
    
    % State derivative function
    cos_Phi = cos(x(3));
    sin_Phi = sin(x(3));
    dX = [x(4)*cos_Phi - x(5)*sin_Phi;...
          x(4)*sin_Phi + x(5)*cos_Phi;...
          x(6);...
          1/car.m*( F_x ) + x(5)*x(6);...
          (1/car.m*( F_y ) - x(4)*x(6));...
          1/car.I_z*( M_z )];
end

%% Dynamic Bicycle Model New

function [dX, alpha_f, alpha_r] = dynamic_bicycle_model_func_new(x,u)
    global car

    % x = [1 - X
    %      2 - Y
    %      3 - Psi
    %      4 - vx
    %      5 - vy
    %      6 - r]

    % u = [1 - d 
    %      2 - delta]

    alpha_f = u(2) - atan2(x(5) + x(6)*car.l_f, x(4));
    alpha_r = -atan2(x(5) - x(6)*car.l_r, x(4));
%     Ff_y = -2*car.D*sin(car.C*atan(car.B*alpha_f-car.E*(car.B*alpha_f-atan(car.B*alpha_f))));
%     Fr_y = -2*car.D*sin(car.C*atan(car.B*alpha_r-car.E*(car.B*alpha_r-atan(car.B*alpha_r))));
    Ff_y = -2*car.D*sin(car.C*atan(car.B*alpha_f));
    Fr_y = -2*car.D*sin(car.C*atan(car.B*alpha_r));
    
    t_bool = (u(1) < 0);
    T = car.T_max + t_bool*(car.T_brake - car.T_max);
    F_prop = 2*car.eta_motor*T*car.GR/car.r_wheel*u(1);
    
    F_roll = car.C_roll*car.m*car.g;
    
    F_drag = 1/2*car.rho*car.C_d*x(4)^2;
    
    cos_delta = cos(u(2));
    F_x = F_prop - F_roll - F_drag - Ff_y*sin(u(2));
    F_y = Ff_y*cos_delta + Fr_y;
    M_z = Ff_y*cos_delta*car.l_f - Fr_y*car.l_r;
    
    % State derivative function
    cos_Phi = cos(x(3));
    sin_Phi = sin(x(3));
    dX = [x(4)*cos_Phi - x(5)*sin_Phi;...
          x(4)*sin_Phi + x(5)*cos_Phi;...
          x(6);...
          1/car.m*( F_x ) + x(5)*x(6);...
          (1/car.m*( F_y ) - x(4)*x(6));...
          1/car.I_z*( M_z )];
end

%% Kinematic Bicycle Model
function dX = kinematic_bicycle_model_func(x,u)
    global car

    % x = [1 - X
    %      2 - Y
    %      3 - Psi
    %      4 - vx
    %      5 - vy
    %      6 - r]

    % u = [1 - d 
    %      2 - delta]

    
    t_bool = (u(1) < 0);
    T = car.T_max + t_bool*(car.T_brake - car.T_max);
    F_prop = 4*car.eta_motor*T*car.GR/car.r_wheel*u(1);
    
    F_roll = car.C_roll*car.m*car.g;
    
    F_drag = 1/2*car.rho*car.C_d*x(4)^2;
    
    F = F_prop - F_roll - F_drag;

    Beta = atan(car.l_r/(car.l_f+car.l_r)*tan(u(2)));
    
    % State derivative function
    cos_Phi = cos(x(3));
    sin_Phi = sin(x(3));
    dX = [x(4)*cos_Phi - x(5)*sin_Phi;...
          x(4)*sin_Phi + x(5)*cos_Phi;...
          sqrt(x(4)^2+x(5)^2)/(car.l_r+car.l_f)*tan(u(2));...
          F/car.m*cos(Beta);...
          F/car.m*sin(Beta);...
          0];
end

function dX = blended_func(x_kin,x_dyn)
    global car

    velocity = sqrt( ((x_kin(4)+x_dyn(4))/2)^2 + ((x_kin(5)+x_dyn(5))/2)^2 );
    lambda_ = min(max((velocity - car.lambda_blend_max)/(car.lambda_blend_max - car.lambda_blend_min), 0), 1);
    
    blended = lambda_*x_dyn + (1-lambda_)*x_kin;

    dX = blended;
end