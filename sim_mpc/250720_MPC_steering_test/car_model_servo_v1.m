function [model] = car_model_servo_v1(x,u,p)

import casadi.*

model = AcadosModel();

model_name = 'SteeringServoModelDisc';

disp(['Model Name: ' model_name]);

% Parâmetros fixos
L = 0.35; % Distância entre eixos
servo_max_rate = 5.2360/10;       % velocidade angular máxima (rad/s)

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

ddelta_request = (delta_cmd - delta) / Ts;


ddelta = if_else(ddelta_request > servo_max_rate, servo_max_rate ,if_else(ddelta_request < -servo_max_rate, -servo_max_rate, ddelta_request));


dvel = (v_in - vel)/Ts;

delta_new= delta + ddelta*Ts;
%delta_new=delta;

delta_avg = (delta_new + delta_cmd)/2;
vel_avg = (v_in + vel)/2; 


% Dinâmica explícita
f_expl = [
    v_in * cos(psi);          % dx/dt
    v_in * sin(psi);          % dy/dt
    (vel / L) * tan(delta);  % dpsi/dt
    ddelta;                    % ddelta/dt
    dvel
];

model.x = x;
model.u = u;
model.p = p;  % ou [] se não usares p
model.f_expl_expr = f_expl;

model.name = 'mpc_model';

end
