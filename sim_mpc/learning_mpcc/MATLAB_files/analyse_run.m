clear; close all
bag_path = '/home/fst/rosbags/autocross_2023-07-24-14-25-28.bag';
h = 0.05;
g = 9.81;
mu_x = 0.9;
mu_y = 0.9;

% Get data from bag
bag = rosbag(bag_path);
bSel = select(bag,'Topic','/control/learning_mpcc/nn_data');
msgStructs = readMessages(bSel,'DataFormat','struct');
for i=1:length(msgStructs)
    r(i) = double(msgStructs{i}.CarVelocity.Velocity.Theta);
    v_x(i) = double(msgStructs{i}.CarVelocity.Velocity.X);
    v_y(i) = double(msgStructs{i}.CarVelocity.Velocity.Y);
    v_x_prev(i) = double(msgStructs{i}.PrevCarVelocity.Velocity.X);
    v_y_prev(i) = double(msgStructs{i}.PrevCarVelocity.Velocity.Y);
    delta(i) = double(msgStructs{i}.ControlCmd.SteeringAngle);
    throttle(i) = double(msgStructs{i}.ControlCmd.Throttle);
    a_x(i) = double(msgStructs{i}.CarAcceleration.X);
    a_y(i) = double(msgStructs{i}.CarAcceleration.Y);
    index(i) = double(msgStructs{i}.Index);
end
vel = sqrt(v_x.^2 + v_y.^2);
Beta = atan(v_y(vel>2.5)./v_x(vel>2.5));
pred_ax = (v_x - v_x_prev) / h;
pred_ay = (v_y - v_y_prev) / h;



%
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
ax_filt = filter(b,a,a_x);
ay_filt = filter(b,a,a_y);

% GG diagram
% figure
% scatter(ax_filt./g,ay_filt./g)
% title('GG-diagram')
% xlabel('a_x')
% ylabel('a_y')

% Pred ax
% figure
% hold on
% plot(ax_filt)
% plot(pred_ax)
% legend('IMU','Pred')
% title('a_x')

% Stability Envelope
% figure
% scatter(r(vel>2.5),rad2deg(Beta))
% title('Stability Envelope')
% xlabel('Yaw Rate')
% ylabel('\beta')

% 
figure
hold on
plot(index(1527:1618),v_x(1527:1618))
plot(index(1527:1618),v_y(1527:1618))
plot(index(1171:1208),v_x(1171:1208))
plot(index(1171:1208),v_y(1171:1208))

figure
hold on
plot(index(1171:1208),delta(1171:1208))
plot(index(1527:1618),delta(1527:1618))
legend('delta_{normal}','delta_{bad}')

%
% figure
% scatter(index,delta)