% Load the rosbag file
bag = rosbag('/home/david/bags/alverca_09_07/trackdrive_2023-07-09-19-23-53.bag');

% Select the desired topic
topic = '/estimation/accel_gravity';

% Extract the message data from the rosbag
msgs = select(bag, 'Topic', topic); 

% Extract the time and signal data from the messages
msgsStruct = readMessages(msgs,'DataFormat','struct');
%%
msgsArray = cell2mat(msgsStruct);
accel_x = zeros(length(msgsArray),1);
for i=1:length(msgsArray)
    accel_x(i) = msgsArray(i).LinearAcceleration.X;
end

% Compute the sampling frequency and the cutoff frequency
fs = 200; % Hz
fc = 20; % Hz

% Compute the filter coefficients
[b,a] = butter(3, fc/(fs/2));

% Apply the filter to the signal
y = filtfilt(b, a, accel_x);

% Plot the original and filtered signals
figure;
plot(accel_x, 'b');
hold on;
plot(y, 'r');
ylabel('Signal');
legend('Original', 'Filtered');

%% Get a throttle

% Select the desired topic
topic = '/control/controller/control_cmd';

% Extract the message data from the rosbag
msgs = select(bag, 'Topic', topic); 

% Extract the time and signal data from the messages
msgsStruct = readMessages(msgs,'DataFormat','struct');
msgsArray = cell2mat(msgsStruct);
throttle = zeros(length(msgsArray),1);
for i=1:length(msgsArray)
    throttle(i) = msgsArray(i).Throttle;
end

%% NN Data
% Select the desired topic
topic = '/control/learning_mpcc/nn_data';

% Extract the message data from the rosbag
msgs = select(bag, 'Topic', topic); 

% Extract the time and signal data from the messages
msgsStruct = readMessages(msgs,'DataFormat','struct');
msgsArray = cell2mat(msgsStruct);
accel_x = zeros(length(msgsArray),1);
accel_y = zeros(length(msgsArray),1);
for i=1:length(msgsArray)
    accel_x(i) = msgsArray(i).CarAcceleration.X;
    accel_y(i) = msgsArray(i).CarAcceleration.Y;
end

figure
scatter(accel_x,accel_y)
figure
hold on
plot(accel_x);
plot(accel_y);
legend('x','y')