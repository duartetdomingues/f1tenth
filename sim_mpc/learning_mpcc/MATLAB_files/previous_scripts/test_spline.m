% Clear
clear all; clc; close all

% Load data
track_data = load('centerline.mat');
% structure_size = length(track_data.X) + length(track_data.Y)
% centerline = struct('X',{},'Y',{});
centerline.X = [track_data.track_data.X; track_data.track_data.X(1)];
centerline.Y = [track_data.track_data.Y; track_data.track_data.Y(1)];
[track_length, progress_list] = calculate_track_length(centerline.X,centerline.Y);

xSpline = spline(progress_list,centerline.X);
ySpline = spline(progress_list,centerline.Y);
dxSpline = fnder(xSpline);
dySpline = fnder(ySpline);

% Plot data
figure
% centerline
plot(centerline.X,centerline.Y);
hold on
% spline
plot(ppval(xSpline,0:309.28),ppval(ySpline,0:309.28),'--');
% spline derivative
x = ppval(xSpline,linspace(0,309.28,25));
y = ppval(ySpline,linspace(0,309.28,25));
P1 = [x' y'];
dx = ppval(dxSpline,linspace(0,309.28,25));
dy = ppval(dySpline,linspace(0,309.28,25));
P2 = [(x+2*dx)' (y+2*dy)'];
arrow3(P1,P2,'k',0.25,0.25,0,1,0.4);
hold off

function [track_length,progress_list] = calculate_track_length(X,Y)
    track_length = 0;
    progress_list = zeros(1,length(X));
    for i=1:length(X)-1
        track_length = track_length + sqrt((X(i+1)-X(i))^2+(Y(i+1)-Y(i))^2);
        progress_list(i+1) = track_length;
    end
end