function [model, constraint, params] = bicycle_model(s0, kapparef, d_left, d_right, stmpc_config, car_config, tire_config)
% bicycle_model.m
% Convertido do Python para MATLAB (CasADi + acados).
% Saídas:
%   model      - struct com símbolos/expressões CasADi e metadados
%   constraint - struct com limites e expressões de constraints não-lineares
%   params     - struct placeholder (valores preenchidos em acados_settings)

import casadi.*

%% ---- Extensão periódica das LUTs -----------------------------------------
len0       = length(s0);
pathlength = s0(end);

% copiar a volta para o início e fim (para bspline periódica)
s0_ext       = [s0(1:end-1)-s0(end), s0, s0(end)+s0(2:end)];
kapparef_ext = [kapparef(1:end-1), kapparef, kapparef(2:end)];
d_left_ext   = [d_left(1:end-1),   d_left,   d_left(2:end)];
d_right_ext  = [d_right(1:end-1),  d_right,  d_right(2:end)];

% Interpolantes (B-spline em s)
kapparef_s  = interpolant('kapparef_s',  'bspline', {s0_ext}, kapparef_ext);
left_bound  = interpolant('left_bound_s','bspline', {s0_ext}, d_left_ext);
right_bound = interpolant('right_bound_s','bspline', {s0_ext}, d_right_ext);

%% ---- Estados e Entradas ---------------------------------------------------
% x = [s; n; theta; v_x; v_y; delta; yaw_rate; accel]
s        = MX.sym('s');
n        = MX.sym('n');
theta    = MX.sym('theta');
v_x      = MX.sym('v_x');
v_y      = MX.sym('v_y');
delta    = MX.sym('delta');
yaw_rate = MX.sym('yaw_rate');
accel    = MX.sym('accel');
x        = vertcat(s, n, theta, v_x, v_y, delta, yaw_rate, accel);
n_x      = length(x);

% u = [jerk; derDelta]
jerk     = MX.sym('jerk');
derDelta = MX.sym('derDelta');
u        = vertcat(jerk, derDelta);
n_u      = length(u);

% variáveis algébricas (nenhuma)
z = vertcat([]);

%% ---- Parâmetros online (passados via ocp.parameter_values) ---------------
V_target      = MX.sym('V_target');
weight_adv    = MX.sym('weight_adv');
weight_qv     = MX.sym('weight_qv');
weight_qn     = MX.sym('weight_qn');
weight_qalpha = MX.sym('weight_qalpha');
weight_qjerk  = MX.sym('weight_qjerk');
weight_ddelta = MX.sym('weight_ddelta');
a_lat_max     = MX.sym('a_lat_max');
a_long_min    = MX.sym('a_long_min');
a_long_max    = MX.sym('a_long_max');
safety_margin = MX.sym('safety_margin');
overtake_d    = MX.sym('overtake_d');
p             = vertcat(V_target, weight_adv, weight_qv, weight_qn, weight_qalpha, ...
                        weight_qjerk, weight_ddelta, a_lat_max, a_long_min, a_long_max, ...
                        safety_margin, overtake_d);

%% ---- Config do veículo e pneus -------------------------------------------
freq  = stmpc_config.MPC_freq;

lr    = car_config.lr;
lf    = car_config.lf;
m     = car_config.m;
Iz    = car_config.Iz;
h_cg  = car_config.h_cg;

mu    = tire_config.friction_coeff;
Bf    = tire_config.Bf;
Cf    = tire_config.Cf;
Df    = tire_config.Df;
Ef    = tire_config.Ef;
Br    = tire_config.Br;
Cr    = tire_config.Cr;
Dr    = tire_config.Dr;
Er    = tire_config.Er;

g = 9.81;

%% ---- Parâmetros do trilho no s actual ------------------------------------
% manter s no intervalo da volta
s_mod = fmod(s, pathlength);  % se necessário, usar rem(s, pathlength)

%% ---- Ângulos de escorregamento e cargas normais ---------------------------
v0      = 5e-1; % viés pequeno para evitar divisão por zero
alpha_f = atan2((-v_y - (lf*yaw_rate)), v_x + v0) + delta;
alpha_r = atan2((-v_y + (lr*yaw_rate)), v_x + v0);

if isfield(stmpc_config, 'load_transfer') && stmpc_config.load_transfer
    F_zf = m*((-accel*h_cg)+(g*lr))/(lf+lr);
    F_zr = m*(( accel*h_cg)+(g*lf))/(lf+lr);
else
    F_zf = m*g*lr/(lf+lr);
    F_zr = m*g*lf/(lf+lr);
end

% Forças laterais (Pacejka)
F_yf = mu * Df * F_zf * sin( Cf*atan(Bf*alpha_f - Ef*(Bf*alpha_f - atan(Bf*alpha_f))) );
F_yr = mu * Dr * F_zr * sin( Cr*atan(Br*alpha_r - Er*(Br*alpha_r - atan(Br*alpha_r))) );

% Forças longitudinais (divisão simples)
F_motor           = accel * m;
motor_split_front = 0.5;
motor_split_rear  = 1 - motor_split_front;
F_xf = motor_split_front * F_motor;
F_xr = motor_split_rear  * F_motor;

%% ---- Dinâmica espacial bicicleta -----------------------------------------
kappa_here = kapparef_s(s_mod);
den        = (1 - kappa_here * n);

s_dot        = ((v_x*cos(theta)) - (v_y*sin(theta))) / den;
n_dot        =  (v_x*sin(theta)) + (v_y*cos(theta));
theta_dot    =  yaw_rate - kappa_here * s_dot;
v_x_dot      = (1/m) * (F_xr + F_xf*cos(delta) - F_yf*sin(delta) + m*v_y*yaw_rate);
v_y_dot      = (1/m) * (F_yr + F_xf*sin(delta) + F_yf*cos(delta) - m*v_x*yaw_rate);
yaw_rate_dot = (1/Iz) * ((-F_yr*lr) + (F_yf*lf)*cos(delta));

% inputs como derivadas para suavidade
delta_dot = derDelta;
accel_dot = jerk;

% dinâmica contínua
f_expl = vertcat( ...
    s_dot, ...
    n_dot, ...
    theta_dot, ...
    v_x_dot, ...
    v_y_dot, ...
    delta_dot, ...
    yaw_rate_dot, ...
    accel_dot);

% função de conveniência
f_expl_func = Function('f_expl_func', {s, n, theta, v_x, v_y, delta, yaw_rate, accel, jerk, derDelta, p}, {f_expl});

%% ---- Estado inicial -------------------------------------------------------
x0 = zeros(n_x,1);

%% ---- Custos ---------------------------------------------------------------
terminal_multiplier = 10;
L0 = weight_qn*(n - overtake_d)^2 + weight_qalpha*theta^2 - weight_qv*(s_dot) ...
     + weight_qjerk*jerk^2 + weight_ddelta*derDelta^2;
L  = L0;
Le = terminal_multiplier * (weight_qn*(n - overtake_d)^2 + weight_qalpha*theta^2 - weight_qv*(s_dot));

if isfield(stmpc_config, 'vy_minimization') && stmpc_config.vy_minimization
    L  = L  + 100*v_y^2;
    Le = Le + terminal_multiplier*(100*v_y^2);
end
if isfield(stmpc_config, 'adv_maximization') && stmpc_config.adv_maximization
    L  = L  - weight_adv * s_dot / freq;
    Le = Le - terminal_multiplier * weight_adv * s_dot / freq;
end

%% ---- Constraints não lineares (track bounds + combinado) -----------------
n_right_bound = n + right_bound(s_mod) - safety_margin;
n_left_bound  = n -  left_bound(s_mod) + safety_margin;

% aceleração lateral (opção)
if isfield(stmpc_config, 'correct_v_y_dot') && stmpc_config.correct_v_y_dot
    a_lat = v_y_dot;
else
    a_lat = v_x*v_x*tan(delta)/(lf+lr);
end

% constraints combinados
combined_constraints = 0;
if isfield(stmpc_config, 'combined_constraints')
    if strcmpi(stmpc_config.combined_constraints, 'ellipse')
        combined_constraints = (a_lat/a_lat_max)^2 + (v_x_dot/a_long_max)^2;
    elseif strcmpi(stmpc_config.combined_constraints, 'diamond')
        combined_constraints = fabs(a_lat/a_lat_max) + fabs(v_x_dot/a_long_max);
    end
end

constraint.expr    = vertcat(n_right_bound, n_left_bound, combined_constraints);
constraint.expr_e  = vertcat(n_right_bound, n_left_bound);
constraint.pathlength = pathlength;

%% ---- Limites copiados da config ------------------------------------------
constraint.delta_min = stmpc_config.delta_min;
constraint.delta_max = stmpc_config.delta_max;
constraint.v_x_min   = stmpc_config.v_min;
constraint.v_x_max   = stmpc_config.v_max;
constraint.a_min     = stmpc_config.a_min;
constraint.a_max     = stmpc_config.a_max;

constraint.ddelta_min = stmpc_config.ddelta_min;
constraint.ddelta_max = stmpc_config.ddelta_max;
constraint.jerk_min   = stmpc_config.jerk_min;
constraint.jerk_max   = stmpc_config.jerk_max;

%% ---- Struct do modelo -----------------------------------------------------
model                 = struct();
model.f_expl_expr     = f_expl;
model.x               = x;
model.n_x             = n_x;
model.u               = u;
model.n_u             = n_u;
model.z               = z;
model.p               = p;
model.name            = 'Spatialbicycle_model';
model.params          = struct();
model.kappa           = kapparef_s;
model.f_expl_func     = f_expl_func;
model.left_bound_s    = left_bound;
model.right_bound_s   = right_bound;
model.track_max       = stmpc_config.track_max_width;

% custos
model.cost_expr_ext_cost_0 = L0;
model.cost_expr_ext_cost   = L;
model.cost_expr_ext_cost_e = Le;

% x0
model.x0 = x0;

% params placeholder (valores atribuídos em acados_settings)
params = struct();

end
