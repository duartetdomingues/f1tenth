clear all; close all; clc

[track_file,track_path] = uigetfile('Tracks/*.json', 'Select a track file');
if isequal(track_file,0)
   error('User selected Cancel');
else
   disp(['User selected ', fullfile(track_path,track_file)]);
end

track_file = split(track_file, '.');

% read track .json file
val = readJSON([track_path, track_file{1}, '.json']);

%% Test track function
figure('Name', 'A track', 'Position', [10 10, 600, 600])

s = 0.001:0.01:310-0.001;
[g_x, g_y, dg_x, dg_y] = track(s);

left = plot([val.X_o; val.X_o(1)], [val.Y_o; val.Y_o(1)], '-mo', 'color', 'b', 'lineWidth', 1.0, 'MarkerSize',3.5, 'MarkerFaceColor','b'); hold on;
right = plot([val.X_i; val.X_i(1)], [val.Y_i; val.Y_i(1)], '-mo', 'color', [255 204 0]/255, 'lineWidth', 1.0,'MarkerSize',3.5, 'MarkerFaceColor',[255 204 0]/255); hold on;

s_start = 5;
[start_g_x, start_g_y,  start_dg_x,  start_dg_y] = track(s_start);

%quiver(start_g_x, start_g_y, start_dg_x, start_dg_y, 'color', 'k');

temp = norm([ start_dg_x,  start_dg_y]);
start_dg_x =  3*start_dg_x/temp;
start_dg_y =  3*start_dg_y/temp;

rot_mat = [0, -1; 1, 0];
dir = 1*rot_mat*[start_dg_x; start_dg_y];

orange_cones_x = [start_g_x + dir(1), start_g_x - dir(1)];
orange_cones_y = [start_g_y + dir(2), start_g_y - dir(2)];

start_line = scatter(orange_cones_x, orange_cones_y, 'o', 'MarkerEdgeColor', [255,140,0]/255, 'MarkerFaceColor',[255,140,0]/255);
start_dir = quiver(orange_cones_x, orange_cones_y, [start_dg_x, start_dg_x], [start_dg_y, start_dg_y], 'color', 'k');


cline = plot(g_x, g_y, '-.', 'color', [144,238,144]/255, 'lineWidth', 1.5); grid minor; axis equal; hold on; xlabel('X [m]'); ylabel('Y [m]');

xhigh = max(max(val.X_o), max(orange_cones_x)) + 2;
xlow = min(min(val.X_o), min(orange_cones_x)) - 2;
yhigh = max(max(val.Y_o), max(orange_cones_y)) + 2;
ylow = min(min(val.Y_o), min(orange_cones_y)) - 2;

xlim([xlow, xhigh]); ylim([ylow, yhigh])

legend([left, right, start_line, start_dir, cline], {'Left limit', 'Right limit', 'Start cones', 'Travel dir.', 'Center line'});