function globalDef
global t_step L STEER_MAX ACC_MAX SPEED_MAX SPEED_MIN GOAL_DIST MAX_T N_X N_U MAX_ITER;
global d_U_MAX DU_TH STOP_SPEED SEARCH_RANGE MIN_COST POSE_ITER Kp;

t_step = 0.2; % Timestep
L = 2.5; % Vehicle wheelbase
STEER_MAX = pi/4; % Max Steering angle
ACC_MAX = 1; % Max acceleration
SPEED_MAX = 200/3.6; % Max Speed55  200
SPEED_MIN = -150/3.6; % Min Speed-20   -150
GOAL_DIST = 10; % Max distance to goal
MAX_T = 3600; % 
N_X = 4; % State vector length
N_U = 2; % Control signal length

MAX_ITER = 4; %
d_U_MAX = pi/12;
DU_TH = 0.1;
STOP_SPEED = 1;
SEARCH_RANGE = 30; % Vehicle position search range
MIN_COST = 100000;
POSE_ITER = 1;
Kp = 1;

% VIENA Parameters
global A g M rw Cd gr Iw Im rho vw Crr1 Crr2 pp
A = 2.14;       %frontal area: m^2
g = 9.8;        %gravitational acceleration: m/s^2
M = 900;        %weight(vehicle+passengers): kg
rw = 0.165;     %wheels radious: m
Cd = 0.33;      %aerodynamic coefficient
gr = 8;         %gearbox ratio
Iw = 0.25;      %wheel inertia: kg.m^2
Im = 0.0025;    %motor's rotor inertia: kg.m^2
rho = 1.225;    %air density: kg/m^3
vw = 0;         %wind speed: m/s
Crr1 = 0.01;    %roll friction coefficient (Vy > 0.1 m/s)
Crr2 = 0;       %roll friction coefficient (Vy < 0.1 m/s)
pp = 2;  

