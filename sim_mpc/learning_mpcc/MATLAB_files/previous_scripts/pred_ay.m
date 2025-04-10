% AY JÁ PODE SER CALCULADO COM AS CONSIDERAÇÕES DE LOAD TRANSFER
% LONGITUDINAIS


% Load the rosbag file
bag = rosbag('/home/david/bags/estoril-13-03-2023/trackdrive_raw_2023-03-13-18-59-42.bag');
T_max_front = 6;
T_max_rear = 12;
T_brake_front = 10;
T_brake_rear = 5;
eta_motor = 1.0;
GR = 15.21;
C_roll = 0.2;
m = 228;
g = 9.81;
rho = 1.225;
C_d = 1.78;
C_l = 2.5;
r_wheel = 0.235;
h_cog = 0.272;
downforce_front = 0.42;
downforce_rear = 0.58;
l_f = 0.66;
l_r = 0.88;
D = 1.0;
C = 2.0;
B = 10.0;

%% Accel
topic = '/estimation/accel_gravity';

% Extract the message data from the rosbag
msgs = select(bag, 'Topic', topic); 

% Extract the time and signal data from the messages
msgsStruct = readMessages(msgs,'DataFormat','struct');
msgsArray = cell2mat(msgsStruct);
accel_y = zeros(length(msgsArray),1);
for i=1:length(msgsArray)
    accel_y(i) = msgsArray(i).LinearAcceleration.Y;
end

% Plot the original and filtered signals
figure;
plot(accel_y(2700:10:end), 'b');
hold on;

%% Get a throttle

% Select the desired topic
topic = '/control/controller/control_cmd';

% Extract the message data from the rosbag
msgs = select(bag, 'Topic', topic); 

% Extract the time and signal data from the messages
msgsStruct = readMessages(msgs,'DataFormat','struct');
msgsArray = cell2mat(msgsStruct);
throttle = zeros(length(msgsArray),1);
steering = zeros(length(msgsArray),1);
for i=1:length(msgsArray)
    throttle(i) = msgsArray(i).Throttle;
    steering(i) = msgsArray(i).SteeringAngle;
end

% Select the desired topic
topic = '/estimation/state_estimation/velocity';

% Extract the message data from the rosbag
msgs = select(bag, 'Topic', topic); 

% Extract the time and signal data from the messages
msgsStruct = readMessages(msgs,'DataFormat','struct');
msgsArray = cell2mat(msgsStruct);
vx = zeros(length(msgsArray),1);
vy = zeros(length(msgsArray),1);
r = zeros(length(msgsArray),1);
for i=1:length(msgsArray)
    vx(i) = msgsArray(i).Velocity.X;
    vy(i) = msgsArray(i).Velocity.Y;
    r(i) = msgsArray(i).Velocity.Theta;
end

%% Pred a_x
throttle_prev = zeros(length(throttle),1);
steering_prev = zeros(length(throttle),1);
for i=1:length(throttle)
    throttle_prev(i+1) = throttle(i);
    steering_prev(i+1) = steering(i);
end

% Braking or Accel Torque
for i=1:length(throttle)
    t_bool = (throttle_prev(1) < 0);
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front);
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear);
    
    % Rolling Resistance
    F_roll = C_roll*m*g; % Considers flat surface
    
    % Drag Force
    F_drag = 1/2*rho*C_d*vx(i)^2;
    
    % Static weight distribution
    Ff_z_static = m * (l_f/(l_f + l_r)) * g;
    Fr_z_static = m * (l_r/(l_f + l_r)) * g;
    
    % Downforce Distribution
    F_downforce = 0.5 * rho * C_l * vx(i)^2;
    
    Ff_downforce = F_downforce * downforce_front;
    Fr_downforce = F_downforce * downforce_rear;
    
    % Slip Angles
    alpha_f = steering_prev(i) - atan2(vy(i) + r(i)*l_f, vx(i));
    alpha_r = atan2(vy(i) - r(i)*l_r, vx(i));
    
    % Longitudinal Load Transfer
    F_f_ay = 2*eta_motor*T_front*GR/r_wheel*throttle_prev(i) - F_drag*downforce_front - F_roll/2;
    Ff_tire_load = (Ff_z_static+Ff_downforce)*D*sin(C*atan(B*alpha_f));
    Fr_tire_load = (Fr_z_static+Fr_downforce)*D*sin(C*atan(B*alpha_r));
    F_y_ay = F_f_ay*sin(steering_prev(i)) + Ff_tire_load*cos(steering_prev(i)) - Fr_tire_load;
    a_y(i) = F_y_ay/m + vx(i)*r(i);
end

plot(a_y);
plot(zeros(1,length(a_y)));