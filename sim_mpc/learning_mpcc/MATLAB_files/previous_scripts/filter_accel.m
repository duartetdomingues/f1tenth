clear;close all;clc
addpath("../yaml_parser/")
config_data =  yaml.loadFile('../../config/default.yaml');
% Load the rosbag file
% bag = rosbag('/home/david/bags/alverca_09_07/trackdrive_2023-07-09-19-23-53.bag');
bag = rosbag('/home/david/bags/alverca_09_07/trackdrive_2023-07-09-19-38-35.bag');

% Select the desired topic
topic = '/control/learning_mpcc/nn_data';

% Extract the message data from the rosbag
msgs = select(bag, 'Topic', topic); 

% Extract the time and signal data from the messages
msgsStruct = readMessages(msgs,'DataFormat','struct');
msgsArray = cell2mat(msgsStruct);
throttle_prev = zeros(length(msgsArray),1);
steering_prev = zeros(length(msgsArray),1);
states = zeros(length(msgsArray),3);
accel_x = zeros(length(msgsArray),1);
accel_y = zeros(length(msgsArray),1);
for i=1:length(msgsArray)
    throttle_prev(i) = msgsArray(i).PrevControlCmd.Throttle;
    steering_prev(i) = msgsArray(i).PrevControlCmd.SteeringAngle;
    states(i,1) = msgsArray(i).CarVelocity.Velocity.X;
    states(i,2) = msgsArray(i).CarVelocity.Velocity.Y;
    states(i,3) = msgsArray(i).CarVelocity.Velocity.Theta;
    accel_x(i) = msgsArray(i).CarAcceleration.X;
    accel_y(i) = msgsArray(i).CarAcceleration.Y;
end

% figure
% scatter(accel_x,accel_y)
% figure
% hold on
% plot(accel_x);
% plot(accel_y);
% legend('x','y')

% Predict accels
car = get_params(config_data);
% accel_x
pred_ax = get_ax(throttle_prev,steering_prev,states(:,1),states(:,2),states(:,3),car);
figure
hold on
yyaxis left
plot(accel_x)
plot(pred_ax,'Color','red')
% accel_y
pred_ay = states(:,1).*states(:,3);
yyaxis right
plot(accel_y)
plot(pred_ay,'Color','black')
legend('ax','pred ax','ay','pred ay')



%%

clear;close all;clc
addpath("../yaml_parser/")
config_data =  yaml.loadFile('../../config/default.yaml');
% Load the rosbag file
% bag = rosbag('/home/david/bags/alverca_09_07/trackdrive_2023-07-09-19-23-53.bag');
bag = rosbag('/home/david/bags/alverca_09_07/trackdrive_2023-07-09-19-38-35.bag');

% Select the desired topic
topic = '/control/learning_mpcc/nn_data';

% Extract the message data from the rosbag
msgs = select(bag, 'Topic', topic); 

% Extract the time and signal data from the messages
msgsStruct = readMessages(msgs,'DataFormat','struct');
msgsArray = cell2mat(msgsStruct);
states = zeros(length(msgsArray),3);
states_prev = zeros(length(msgsArray),3);
accel_x = zeros(length(msgsArray),1);
accel_y = zeros(length(msgsArray),1);
for i=1:length(msgsArray)
    states(i,1) = msgsArray(i).CarVelocity.Velocity.X;
    states(i,2) = msgsArray(i).CarVelocity.Velocity.Y;
    states(i,3) = msgsArray(i).CarVelocity.Velocity.Theta;
    states_prev(i,1) = msgsArray(i).PrevCarVelocity.Velocity.X;
    states_prev(i,2) = msgsArray(i).PrevCarVelocity.Velocity.Y;
    states_prev(i,3) = msgsArray(i).PrevCarVelocity.Velocity.Theta;
    accel_x(i) = msgsArray(i).CarAcceleration.X;
    accel_y(i) = msgsArray(i).CarAcceleration.Y;
end

% 
pred_ax = (states(:,1)-states_prev(:,1))./0.05 - states(:,2).*states(:,3);
pred_ay = (states(:,2)-states_prev(:,2))./0.05 + states(:,1).*states(:,3);

figure
hold on
plot(accel_x)
plot(pred_ax,'Color','red')
legend('ax','pred ax')
% accel_y
figure 
hold on
plot(accel_y)
plot(pred_ay,'Color','black')
legend('ay','pred ay')


%% Functions
function car = get_params(config_data)
    car.l_f = config_data.model_params.l_f;
    car.l_r = config_data.model_params.l_r;
    car.m = config_data.model_params.m;
    car.I_z = config_data.model_params.I_z;
    car.T_max_front = config_data.model_params.T_max_front;
    car.T_max_rear = config_data.model_params.T_max_rear;
    car.T_brake_front = config_data.model_params.T_brake_front;
    car.T_brake_rear = config_data.model_params.T_brake_rear;
    car.GR = config_data.model_params.GR;
    car.eta_motor = config_data.model_params.eta_motor;
    car.r_wheel = config_data.model_params.r_wheel;
    car.g = config_data.model_params.g;
    car.C_roll = config_data.model_params.C_roll;
    car.rho = config_data.model_params.rho;
    car.lambda_blend_min = config_data.model_params.lambda_blend_min;
    car.lambda_blend_max = config_data.model_params.lambda_blend_max;
    car.C_d = config_data.model_params.C_d;
    car.C_l = config_data.model_params.C_l;
    car.downforce_front = config_data.model_params.C_d;
    car.downforce_rear = config_data.model_params.C_l;
    car.lambda_blend_min = config_data.model_params.lambda_blend_min;
    car.lambda_blend_max = config_data.model_params.lambda_blend_max;
    % Define Tyre parameters
    car.B = config_data.tyre_params.B;
    car.C = config_data.tyre_params.C;
    car.D = config_data.tyre_params.D;
end

function pred_ax = get_ax(throttle,steering,vx,vy,r,car)
    % Braking or Accel Torque
    t_bool = (throttle < 0);
    T_front = (~t_bool)*car.T_max_front + t_bool*(car.T_brake_front);
    T_rear = (~t_bool)*car.T_max_rear + t_bool*(car.T_brake_rear);

    % Rolling Resistance
    F_roll = car.C_roll*car.m*car.g;
    
    % Drag Force
    F_drag = 1/2*car.rho*car.C_d*vx.^2;

    % Longitudinal Load Transfer
    F_f_ax = 2*car.eta_motor.*T_front*car.GR/car.r_wheel.*throttle;
    F_r_ax = 2*car.eta_motor.*T_rear*car.GR/car.r_wheel.*throttle;
    pred_ax = (F_f_ax.*cos(steering) + F_r_ax - F_roll - F_drag)/car.m - vy.*r;
end