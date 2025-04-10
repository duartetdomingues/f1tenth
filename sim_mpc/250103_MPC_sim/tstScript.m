function tstScript
close all
clear all

load('refXsaved.mat')
load('refYsaved.mat')

path_tag = '13_04_21';
ref_path = input_scenario_V3(path_tag);

figure(1)
plot(ref_path(:,1), ref_path(:,2))
hold on

for i=1:length(saveXref)
    plot(saveXref(i,:),saveYref(i,:))
    hold on
end
