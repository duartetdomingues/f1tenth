function [model] = car_model_v1(x,u,p)

import casadi.*

model = AcadosModel();

model_name = 'Car_model_v1';

disp(['Model Name: ' model_name]);


% Parâmetros fixos
servo_max_rate = 5.2360;       % velocidade angular máxima (rad/s)
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

f_expl = [ ...
        u(2)*cos(x(3));        % dx/dt
        u(2)*sin(x(3));        % dy/dt
        (u(2)/L)*tan(u(1));    % dpsi/dt
        0;
        0];                    % dtheta e dv são nulos (v é entrada direta)


% Dinâmica implícita
%f_impl = f_expl - xdot;

% Model struct
model.x = x;
%model.xdot = xdot;
model.u = u;
model.p = p;
model.f_expl_expr = f_expl;
%model.f_impl_expr = f_impl;
model.name = model_name;

end
