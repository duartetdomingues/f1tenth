function [model] = car_model_servo_v3_tau(x,u,p)

import casadi.*

model = AcadosModel();

model_name = 'SteeringServoModel';

disp(['Model Name: ' model_name]);

% Parâmetros fixos
servo_max_rate = 5.2360/10;       % velocidade angular máxima (rad/s)
L = 0.35; % Distância entre eixos
tau  = 0.164 ;%0.1148; % 0.164


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

delta_next = delta + (Ts/tau)*(delta_cmd - delta);     % 1ª ordem

ddelta = (delta_next - delta)/Ts;

dvel = (v_in - vel)/Ts;

% Dinâmica explícita
f_expl = [
    v_in * cos(psi);          % dx/dt
    v_in * sin(psi);          % dy/dt
    (v_in / L) * tan(delta);  % dpsi/dt
    ddelta;                    % ddelta/dt
    dvel
];
    
% Dinâmica implícita
%f_impl = f_expl - xdot;

% Model struct
model.x = x;
%model.xdot = xdot;
model.u = u;
model.p = p;
model.f_expl_expr = f_expl;

% model.con_h_expr = ddelta_expr;
% model.con_h_expr_0 = ddelta_expr;

%model.f_impl_expr = f_impl;
model.name = 'mpc_model';

end
