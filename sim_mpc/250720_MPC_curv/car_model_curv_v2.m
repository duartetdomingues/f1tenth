function [x_next] = car_model_curv_v2(Ds, track,x,u)

import casadi.*

%filename = params()

kappa_lut =interpolant('kappa_lut','linear',{track.s_traj},track.kappa_traj);
n_l_lut =interpolant('n_l_lut','linear',{track.s_traj},track.nl_traj);
n_r_lut =interpolant('n_r_lut','linear',{track.s_traj},track.nr_traj);




% p = sx.sym('p', np);
% 
% kappa = p(1);
% nl_s = p(2);
% nr_s = p(3);

% Bf = params(1);
% Br = params(2);
% Cf = params(3);
% Cr = params(4);
% Df = params(5);
% Dr = params(6);
% params = [Bf;Br;Cf;Cr;Df;Dr];

% Bf = 7.6671;
% Br = 7.1036;
% Cf = 1.2628;
% Cr = 1.7356;
% Df = 1.2307;
% Dr = 1.0252;

Bf = 5.5405;
Br = 6.6014;
Cf = 0.4182;
Cr = 0.5624;
Df = 1.8635;
Dr = 1.7327;
Ef = 0.3722;
Er = 1.0648;
params = [];




% Parâmetros fixos
servo_max_rate = 5.2360;       % velocidade angular máxima (rad/s)
Wheelbase = 0.324; % Distância entre eixos

mass = 3;
track_width =0.296;
length = 0.5;

% Constant motor Force-Cm= Fx/T : T=0.20 , acc_x= 5 m/s2 m=3 -> F=15N -> 75 
Cm = 75;

% rolling resistance force Frr - Desacelaration when stop Duty =
% acc_x=1.5 m/s2 F= 4.5N
Frr = 4.5;
epsilon = 1e-3;

% Tires friction ellipse params
rho_long = 1.2;       % escolheu-se ρ>1 → reduz Fx_max
lambda_f = 0.9;       % 10 % de margem (frente)
lambda_r = 0.9;       % idem (trás)

q_beta  = 0.05;               % peso para o ângulo de deslize

g=9.8;
Lf= Wheelbase/2;
Lr= Wheelbase/2;

% Rotational inertia
%Iz=1/12*mass*(0.5^2 + track_width^2) %0.5 are the car length
Iz=length*mass*Wheelbase^2; % 0.3 are de mass disturbation


% Estados x = [s; n; µ; vx; vy; r; δ; T]
s           = x(1); % s
n           = x(2); % n
heading_u   = x(3); % µ
vx         = x(4); % vx
vy         = x(5); % vy
yaw_rate    = x(6); % r
delta       = x(7); % δ
throttle    = x(8); % T

% Entradas
d_delta = (u(1)-delta)/Ds;
d_throttle  = (u(2)-throttle)/Ds;

% Normal loads on the tires
Fn_f = Lr/(Lf+Lr) *mass *g;   
Fn_r = Lf/(Lf+Lr) *mass *g;   

% Slip angles at the wheels
vx_safe = max(abs(vx), 0.1); % Evita divisão por zero  
alfa_f = atan((vy + Lf* yaw_rate)/vx_safe)-delta; 
alfa_r = atan((vy - Lr* yaw_rate)/vx_safe); 
% alfa_f = 0.01;
% alfa_r = 0.01;

% Lateral tire forces
Fy_f = Fn_f * Df * sin(Cf * atan(Bf*alfa_f)); 
Fy_r = Fn_r * Dr * sin(Cr * atan(Br*alfa_r));

% Force Motor
Fm=Cm*throttle;

sign_vx = vx / sqrt(vx^2 + epsilon);

% Force x
Fx = Fm - Frr * sign_vx;

kappa= kappa_lut (s);
nl_s= n_l_lut(s);
nr_s = n_r_lut(s);

% kappa= 0.38;
% nl_s=1;
% nr_s = 1;

disp([kappa,nl_s, nr_s])

d_s = vx*cos(heading_u) - vy*sin(heading_u)/(1-n*kappa);

d_n = vx*sin(heading_u) + vy*cos(heading_u);

d_u = yaw_rate - kappa*d_s;

d_vx = 1/mass*(Fx - Fy_f * sin(delta) + mass*vy*yaw_rate);

d_vy = 1/mass*(Fy_r + Fy_f * cos(delta) - mass*vx*yaw_rate);

d_r = 1/Iz * (Fy_f * Lf * cos(delta) - Fy_r* Lr);


f_expl = [ ...
        d_s;
        d_n;
        d_u;
        d_vx;
        d_vy;
        d_r;
        d_delta;
        d_throttle
        ];




x_next=x+Ds*f_expl;

end
