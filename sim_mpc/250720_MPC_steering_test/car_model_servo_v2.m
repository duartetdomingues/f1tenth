function [model] = car_model_servo(x,u,p)

import casadi.*

model = AcadosModel();

model_name = 'SteeringServoModel';

disp(['Model Name: ' model_name]);

% Parâmetros fixos
servo_max_rate = 5.2360/10;       % velocidade angular máxima (rad/s)
L = 0.35; % Distância entre eixos


% Estados
x_pos   = x(1);
y_pos   = x(2);
psi     = x(3);
delta   = x(4);
vel     = x(5);

% Entradas
delta_cmd = u(1);
v_in      = u(2);

Ts=p(1);


% % Derivadas dos estados
% x_dot = MX.sym('x_dot');
% y_dot = MX.sym('y_dot');
% psi_dot = MX.sym('psi_dot');
% delta_dot = MX.sym('delta_dot');
% xdot = vertcat(x_dot, y_dot, psi_dot, delta_dot);

% Saturação da derivada de delta


servo_max_rate = 5.2360/10;       % velocidade angular máxima (rad/s)
ddelta_request = (delta_cmd - delta) / Ts;
ddelta = if_else(ddelta_request > servo_max_rate, servo_max_rate ,if_else(ddelta_request < -servo_max_rate, -servo_max_rate, ddelta_request));



dvel = (v_in - vel)/Ts;

%delta_new= delta + ddelta*p(1);
delta_new=delta;

%delta_avg = (delta_new + delta_cmd)/2;
vel_avg = (v_in + vel)/2; 


% Dinâmica explícita
f_expl = [
    vel * cos(psi);          % dx/dt
    vel * sin(psi);          % dy/dt
    (vel / L) * tan(delta);  % dpsi/dt
    ddelta_request;                    % ddelta/dt
    dvel
];

delta = delta_cmd;


ddelta_expr = (delta_cmd - delta) / Ts;  % ddelta = (delta_cmd - delta) / Ts
    
% Dinâmica implícita
%f_impl = f_expl - xdot;

% Model struct
model.x = x;
%model.xdot = xdot;
model.u = u;
model.p = p;
model.f_expl_expr = f_expl;

model.con_h_expr = ddelta_expr;
model.con_h_expr_0 = ddelta_expr;

%model.f_impl_expr = f_impl;
model.name = 'mpc_model';

end
