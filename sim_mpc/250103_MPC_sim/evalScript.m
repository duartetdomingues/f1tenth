function evalScript(path_tag)
close all
if nargin < 1
%     path_tag = '13_04_21';
    path_tag = '05_05_21';
    pitch_tag = 'flat';
    slipcheck = 1;
end

log_str = strcat('res/log_', path_tag, '_', pitch_tag, '.mat');
load(log_str)

% Compute pose and speed error and acc and steering change rate
for i = 1:length(simX)
    if i <= length(simX)-1
        steerCR(i) = simU(i,2) - simU(i+1,2);
        accCR(i) = simU(i,1) - simU(i+1,1);
    end
end
steerCR = [0 steerCR];
maxposeError = max(evalpose.poseError);
minposeError = min(evalpose.poseError);
meanposeError = mean(evalpose.poseError);

maxvError = max(evalpose.vError);
minvError = min(evalpose.vError);
meanvError = mean(evalpose.vError);


fprintf('POSE ERROR \n');
fprintf('Max Error: %d \t Min Error: %d \t Mean Error: %d \n',maxposeError, minposeError, meanposeError);
fprintf('SPEED ERROR \n');
fprintf('Max Error: %d \t Min Error: %d \t Mean Error: %d \n',maxvError, minvError, meanvError);

ET_exp = sum(evalpose.eKinSP)/3600000;
ET_real = sum(evalpose.eKin)/3600000;
rel_error_ET = (abs(ET_exp - ET_real)/ET_exp)*100;

t_star = t_sp(end);
t_real = simTime(end);
rel_error_t = (abs(t_star - t_real)/t_star)*100;

fprintf('ENERGY Values \n');
fprintf('Expected: %d \t Real: %d \t Rel Error: %d \n', ET_exp, ET_real, rel_error_ET);
fprintf('TIME Values\n');
fprintf('Expected: %d \t Real: %d \t Rel Error: %d \n',t_star, t_real, rel_error_t);

accUpper = 1*ones(1, length(accCR));
accLower = -1*ones(1, length(accCR));
steerUpper = (pi/4)*ones(1, length(steerCR));
steerLower = (-pi/4)*ones(1, length(steerCR));
speedUpper = (55/3.6)*ones(1,round(t_sp(end-1)));
speedLower = (-20/3.6)*ones(1,round(t_sp(end-1)));
steerCRUpper = (pi/12)*ones(1, length(steerCR));
steerCRLower = (-pi/12)*ones(1, length(steerCR));

% Compute vehicle traveled distance
xDiff= diff(evalpose.x);
yDiff= diff(evalpose.y);
s = sqrt(xDiff.*xDiff + yDiff.*yDiff);
for i=2:length(s)
    s(i)= s(i-1)+s(i);
end
s = [0 s];
steerUpper2 = (pi/4)*ones(1, length(s));
steerLower2 = (-pi/4)*ones(1, length(s));
steerCRUpper2 = (pi/12)*ones(1, length(s));
steerCRLower2 = (-pi/12)*ones(1, length(s));

% PATH comparison
figure(1)
plot(ref_path(:,1), ref_path(:,2), 'LineWidth', 2)
hold on
plot(simX(1:end-1,1), simX(1:end-1,2), 'LineWidth',2)
grid on
legend('Reference Path','Vehicle Path')
xlabel('X [m]')
ylabel('Y [m]')

% SPEED comparison
figure(2)
plot(t_sp, sp, 'k')
hold on 
plot(simTime, simX(:,3), 'b', 'LineWidth', 2)
grid on
plot(speedUpper, 'r', 'LineWidth', 2)
plot(speedLower, 'r', 'LineWidth', 2)
legend('Reference Speed','Vehicle Speed','Speed Limit')
xlabel('Time [s]')
ylabel('Speed [m/s]')

% YAW comparison
figure(3)
plot(ref_path(:,3),'k')
hold on
plot(simX(:,4),'b', 'LineWidth',2)
grid on
legend('Reference Yaw','Vehicle Yaw')
xlabel('Point')
ylabel('Yaw')


% ACC limits
figure(4)
plot(simU(:,1), 'k')
hold on
plot(accUpper, 'r', 'LineWidth', 2)
plot(accLower, 'r', 'LineWidth', 2)
grid on
legend('Acceleration','Acceleration limit')
xlabel('Point')
ylabel('Acceleration [m/s^2]')

% STEER limits
figure(5)
plot(s, simU(:,2), 'k', 'LineWidth',2)
hold on
plot(s, steerUpper2, 'r', 'LineWidth', 2)
plot(s, steerLower2, 'r', 'LineWidth', 2)
grid on 
legend('Steering angle', 'Steering angle limit')
xlabel('Distance Traveled[m]')
ylabel('Steering angle [rad]')

%Acceleration change rate -> Measure of comfort
figure(6)
plot(accCR, 'k')
grid on 
legend('Acceleration Change Rate')
xlabel('Point')
ylabel('Acceleration Change Rate [m/s^2]')

%Steering change rate -> Measure of comfort
figure(7)
plot(s,steerCR, 'k', 'LineWidth', 2)
hold on
grid on
plot(s,steerCRUpper2, 'r', 'LineWidth', 2)
plot(s, steerCRLower2, 'r', 'LineWidth', 2)
legend('Steering Change Rate', 'St. CR Limits')
xlabel('Distance Traveled[m]')
ylabel('Steering Change Rate [rad]')

figure(8)
plot(ref_path(:,4), sp, 'LineWidth', 2)
hold on
grid on
plot(s,evalpose.v, 'LineWidth', 2)
legend('Reference Speed','Vehicle Speed')
xlabel('Distance Traveled [m]')
ylabel('Speed [m/s]')

if ~strcmp(pitch_tag, 'flat')
    x = ref_path(:,1);
    y = ref_path(:,2);
    for i = 1:length(ref_path)
    if i == 1
        z(i) = ref_path(i,4)*sind(ref_path(i,6));
    else
        z(i) = z(i-1)+((ref_path(i,4)-ref_path(i-1,4))*sind(ref_path(i,6)));
    end
    end
    figure(9)
    clf
    plot3(x,y,z, 'LineWidth',2)
    hold on
    grid on
    scatter3(evalpose.x, evalpose.y, evalpose.z)
    plot(x,y)
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')
    legend('Reference Path','Vehicle Path','2D Reference Path')
end

if slipcheck == 1
    checkSlip(ref_path, sp)
end
% Build data to use in VIENA gif simulation
arrays.x = simX(:,1);
arrays.y = simX(:,2);
arrays.v = simX(:,3);
arrays.yaw = simX(:,4);
arrays.steer = simU(:,2);

save('carState_.mat', 'arrays')
save('ref_path130421_.mat', 'ref_path')


% Evaluate the lack of slip conditions on the speed profiler maybe later,
% meaning, generate max speed before slip conditions happen and plot this
% agaisnt the vehicle speed.





