%% Speed and Steering Control for Autonomous Electric Vehicles
%   Fábio Portelinha    2021/2022   IST
%
%
%   Supervisors:    José Gaspar
%                   João Fernandes
%

function [model, constraint] = car_model()

import casadi.*
model = struct();
constraint = struct();

model_name = 'car_model';


%% Car parameters
L = 2.5;

%% CasADi Model
% set up states & controls
x = MX.sym('x');
y = MX.sym('y');
v = MX.sym('v');
theta = MX.sym('theta');

sym_x= vertcat(x, y, v, theta);

% controls
acc = MX.sym('acc');
delta = MX.sym('delta');
u = vertcat(acc, delta);

% xdot
xdot = MX.sym('xdot');
ydot = MX.sym('ydot');
vdot = MX.sym('vdot');
thetadot = MX.sym('thetadot');
sym_xdot = vertcat(xdot, ydot, vdot, thetadot);

% algebraic variables
z = [];

% parameters
p = [];

% constraint on forces
% a_lat = C2 * v * v * delta + Fxd * sin(C1 * delta) / m;
% a_long = Fxd / m;

% Model bounds
% model.n_min = -0.12;  % width of the track [m]
% model.n_max = 0.12;  % width of the track [m]

% state bounds
model.acc_min = -1.0;
model.acc_max = 1.0;

model.delta_min = -pi/4;  % minimum steering angle [rad]
model.delta_max = pi/4;  % maximum steering angle [rad]

% input bounds
% model.ddelta_min = -0.5;  % minimum change rate of stering angle [rad/s]
% model.ddelta_max = 0.5;  % maximum change rate of steering angle [rad/s]
% model.dthrottle_min = -10;  % -10.0  % minimum throttle change rate
% model.dthrottle_max = 10;  % 10.0  % maximum throttle change rate

% nonlinear constraint
% constraint.alat_min = -4;  % maximum lateral force [m/s^2]
% constraint.alat_max = 4;  % maximum lateral force [m/s^1]
% 
% constraint.along_min = -4;  % maximum lateral force [m/s^2]
% constraint.along_max = 4;  % maximum lateral force [m/s^2]

% Define initial conditions
model.x0 = [0, 0, 0, 0];

% define constraints struct
% constraint.alat = Function('a_lat', {x, u}, {a_lat});
% constraint.pathlength = pathlength;
% constraint.expr = vertcat(a_long, a_lat, n, D, delta);

% Define model struct
% params = struct();
% params.C1 = C1;
% params.C2 = C2;
% params.Cm1 = Cm1;
% params.Cm2 = Cm2;
% params.Cr0 = Cr0;
% params.Cr2 = Cr2;
% model.f_expl_expr = vertcat((v*cos(theta))-(v*theta*sin(theta)), ...
%     (v*sin(theta))+(v*theta*cos(theta)), acc, ...
%     ((v*tan(delta))/L)+((v*delta)/(L*(cos(delta))^2)));
model.f_expl_expr = vertcat((v*cos(theta)), ...
    (v*sin(theta)), acc, ...
    ((v*tan(delta))/L));
model.x = sym_x;
model.xdot = sym_xdot;
model.u = u;
model.z = z;
model.p = p;
model.name = model_name;
% model.params = params;

end
