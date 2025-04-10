close all
% Choose and read config file
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
% Define Tyre parameters
car.B = config_data.tyre_params.B;
car.C = config_data.tyre_params.C;
car.D = config_data.tyre_params.D;
% car.E = config_data.tyre_params.E;

size = 10000;
history = zeros(6,size);

x = [0 0 0 0 0 0]'; %xinit
u = [1, 0.1]';
figure;
hold on;
for i = 1:size
    dx = kinematic_bicycle_model_func(x,u);
    x = x + 0.01*dx;
    x(6) = dx(3);
    history(:,i) = x;
end
%%
tiledlayout(3,2)
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
hold off;


%% Dynamic Bicycle Model
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
