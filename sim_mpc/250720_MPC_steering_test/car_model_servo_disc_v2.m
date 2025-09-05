function [model] = car_model_servo_disc_v2(x,u,p)

import casadi.*

model = AcadosModel();

model_name = 'SteeringServoModelDisc';

disp(['Model Name: ' model_name]);

% Parâmetros fixos
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
%servo_max_rate = p(2);

x_next = [
        x(1) + Ts * x(5) * cos(x(3));
        x(2) + Ts * x(5) * sin(x(3));
        x(3) + Ts * (x(5)/L) * tan(x(4));
        u(1);  % delta = delta_cmd
        u(2);  % vel = v_in
    ];

ddelta_expr = (delta_cmd - delta) / Ts;  % ddelta = (delta_cmd - delta) / Ts

model.x = x;
model.u = u;
model.p = p;  % ou [] se não usares p
model.disc_dyn_expr = x_next;  % <------ usa esta dinâmica discreta!

model.con_h_expr = ddelta_expr;
model.con_h_expr_0 = ddelta_expr;


model.name = 'mpc_model';

end
