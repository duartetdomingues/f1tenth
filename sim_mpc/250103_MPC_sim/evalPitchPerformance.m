function evalPitchPerformance

if nargin < 1
    path_tag = '13_04_21';   
%     path_tag = 'lin_ret'; 
end

ref_path = input_scenario_V3(path_tag);
ref_path = add_path_theta( ref_path );

vState_str = strcat('res/vehicleState10_', path_tag, '.mat');
vControl_str = strcat('res/vehicleControl10_', path_tag, '.mat');
sRef_str = strcat('res/speedProfile10_', path_tag, '.mat');
time_str = strcat('res/simTime10_', path_tag, '.mat');
spT_str = strcat('res/speedProfiletime10_', path_tag, '.mat');
eval_str = strcat('res/evalStats10_', path_tag, '.mat');

load(vState_str,'simX')
load(vControl_str,'simU')
load(sRef_str, 'sp')
load(time_str, 'simTime')
load(spT_str, 't_sp')
load(eval_str, 'evalpose')

maxTimeidx = find(simTime >= 40);

simX = simX(1:maxTimeidx(1),:);
simTime = simTime(1:maxTimeidx(1));


intSpeed = interp1(simTime,simX(:,3),t_sp);
 
vError = intSpeed(1:177) - sp(1:177);
minVerror = min(abs(vError));
maxVerror = max(abs(vError));
meanVerror = abs(mean((vError)));


last_idx = 1;
for i = 1:length(evalpose.x) 
    state.x = evalpose.x(i);
    state.y = evalpose.y(i);
    
    closeIdx = calc_nearest_path_index(state, ref_path, last_idx);
    last_idx = closeIdx;
   
    poseError(i) = norm([state.x state.y] - [ref_path(last_idx,1) ref_path(last_idx,2)]);
    if last_idx >=174
        break
    end
end

minPerror = min(poseError);
maxPerror = max(poseError);
meanPerror = mean(poseError);

x = ref_path(:,1);
y = ref_path(:,2);
for i = 1:length(ref_path)
    if i == 1
        z(i) = ref_path(i,4)*sind(ref_path(i,6));
    else
        z(i) = z(i-1)+(ref_path(i,4)*sind(ref_path(i,6)));
    end
end

evalpose.x = evalpose.x(1:maxTimeidx(1));
evalpose.y = evalpose.y(1:maxTimeidx(1));
evalpose.z = evalpose.z(1:maxTimeidx(1));
figure(1)
clf
plot3(x,y,z)
hold on
grid on
scatter3(evalpose.x, evalpose.y, evalpose.z)
plot(x,y)