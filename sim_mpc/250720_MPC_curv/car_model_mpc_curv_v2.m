function [model , constraints] = car_model_mpc_curv_v2(Ts, R, track)

import casadi.*

model = AcadosModel();

model_name = 'Car_model_v1';

disp(['Model Name: ' model_name]);

nx = 5;  % Estados: [s; n; µ; vx; vy; r; δ; T]
nu = 2;  % Controlo: [delta, v]
%nparams =6 ; % Parâmetros: [Bf,Br,Cf,Cr,Df,Dr]


x = SX.sym('x', nx);
u = SX.sym('u', nu);
% nl_s = SX.sym('nl', ns);
% nr_s = SX.sym('nr', ns);
%params = SX.sym('p_global', nparams);

%filename = params()

% acrescentar no fim: s0(end) + s0(2:end)
track.s_traj = [ track.s_traj ; track.s_traj(end) + track.s_traj(2:end)];
    
% acrescentar no início: s0(1:end-1) - s0(end)
track.s_traj = [ track.s_traj(1:end-1) - track.s_traj(end) ; track.s_traj ];


track.kappa_traj = [ track.kappa_traj(1:end-1) - track.kappa_traj(end) ; track.kappa_traj ];
track.kappa_traj = [ track.kappa_traj(1:end-1) - track.kappa_traj(end) ; track.kappa_traj ];

track.nl_traj = [ track.nl_traj(1:end-1) - track.nl_traj(end) ; track.nl_traj ];
track.nl_traj = [ track.nl_traj(1:end-1) - track.nl_traj(end) ; track.nl_traj ];

track.nr_traj = [ track.nr_traj(1:end-1) - track.nr_traj(end) ; track.nr_traj ];
track.nr_traj = [ track.nr_traj(1:end-1) - track.nr_traj(end) ; track.nr_traj ];

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

% Bf = 5.6671;
% Br = 5.1036;
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


% Estados x = [s; n; µ; vx; vy; r; δ; T]
s           = x(1); % s
n           = x(2); % n
heading_u   = x(3); % µ
vx         = x(4); % vx
yaw_rate    = x(5); % r

throttle=u(2);
delta=u(1);

% Force Motor
Fm=Cm*throttle;


% Force x
Fx = Fm;

kappa= kappa_lut (s);
% nl_s= n_l_lut(s);
% nr_s = n_r_lut(s);
%kappa=0.01;

% kappa= 0.38;
nl_s=1;
nr_s = 1;


%d_s = (vx*cos(heading_u) - vy*sin(heading_u))/(1-n*kappa);
d_s = (vx*cos(heading_u))/(1 - kappa*n);

d_n = vx*sin(heading_u);

d_u = yaw_rate - kappa*d_s;

d_vx = 1/mass * Fx;

d_r = vx*tan(delta)/Wheelbase;


f_expl = [ ...
        d_s;
        d_n;
        d_u;
        d_vx;
        d_r;
        ];

% 
% % state bounds
constraints.throttle_min = -0.3;
constraints.throttle_max = 0.3;
constraints.throttle_idx = 6; % idx zero-based

constraints.delta_min = -0.40;  % minimum steering angle [rad]
constraints.delta_max = 0.40;  % maximum steering angle [rad]
constraints.delta_idx = 5; % idx zero-based
% 
% % input bounds
% constraints.ddelta_min = -2.0;  % minimum change rate of stering angle [rad/s]
% constraints.ddelta_max = 2.0;  % maximum change rate of steering angle [rad/s]
% constraints.dthrottle_min = -10;  % -10.0  % minimum throttle change rate
% constraints.dthrottle_max = 10;  % 10.0  % maximum throttle change rat

% ----- ângulos de deslize ---------------------------------------
% beta_dyn = atan2(vy,vx);
% beta_kin = atan( delta * Lr / (Lf+Lr) );
% Bexpr    = q_beta * (beta_dyn - beta_kin)^2;

% ----- termo de rate-penalty ------------------------------------
uexpr    = u.' * R * u;

epsilon_cost = 1e-10;
% L_stage = -8*(Ts*d_s) %+ 40*n^2+ 1*heading_u^2 + uexpr %+ Bexpr;     % (1×1) OK
% L_stage_e = -8*(Ts*d_s) %+ 40*n^2+ 1*heading_u^2 %+ Bexpr;
% 
% model.cost_expr_ext_cost = L_stage;
% model.cost_expr_ext_cost_0 = L_stage;
% model.cost_expr_ext_cost_e = L_stage_e;  % custo terminal = 0



%{
% pesos de estabilização (ajusta à vontade)
Qreg = diag([0, 40, 40, 0, 10, 4,  0, 0]);   % penaliza n, mu, vy, r
Rreg = diag([0.5, 1.5]);                    % penaliza u

% den protegido


 s_min =0;
s_max= track.s_traj(end);
kap  = kappa_lut( min( max( x(1), s_min ), s_max ) );
den  = 1 - kap*x(2);
den  = 0.5*(den + sqrt(den*den + 1e-6));    % clamp suave
sdot = ( x(4)*cos(x(3)) - x(5)*sin(x(3)) ) / den;

w_prog = 8;            % começa baixo
w_term = 4;

stage_cost = x.'*Qreg*x + u.'*Rreg*u - w_prog*(Ts*sdot);
term_cost  = x.'*Qreg*x            - w_term*x(1);

model.cost_expr_ext_cost_0 = stage_cost;
model.cost_expr_ext_cost   = stage_cost;
model.cost_expr_ext_cost_e = term_cost; 
%}



%p=[];

model.f_expl_expr = f_expl;
model.x = x;
model.u = u;
%model.p = p;
%model.p_global = params;


% Dinâmica implícita
%f_impl = f_expl - xdot;

% Model struct

%model.f_impl_expr = f_impl;
model.name = 'mpc_model';

end
