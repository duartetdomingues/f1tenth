%% ----------   Initialization  -----------------------------------------
clear all; clc; close all
% Add yaml parser to path
addpath("yaml_parser/")

% Downsample of centerline for spline
DOWNSAMPLE_RATE = 40; % Put to zero if no downsample is needed
global EXTEND_CENTERLINE;
EXTEND_CENTERLINE = 0;

%% ----------   Select track file   -------------------------------------

% [track_file,track_path] = uigetfile('Tracks/*.json;*.sdf;*.yaml', 'Select a track file');
% track_file = 'acceleration.yaml';
% track_file = 'acceleration_2_points.yaml';
% track_file = 'trackdrive.yaml';
track_file = 'estoril_13_03_third_try_again.yaml'
track_path = '/home/david/fst/autonomous-systems/src/control/learning_mpcc/MATLAB_files/Tracks/';
if isequal(track_file,0)
   error('User selected Cancel');
else
   disp(['User selected ', fullfile(track_path,track_file)]);
end

track_file = split(track_file, '.');
order_cones = 1;

% Read in case of sdf
if strcmp(track_file{2}, 'sdf')
    readSDF([track_path, track_file{1}], false);
    order_cones = 0;
end

% create output directory
folder_name = ['output_', track_file{1}, '_', 'MPCC'];

try
    disp('Overwriting previous version of this tracks solver');
    rmdir(folder_name,'s');
catch
    disp('Creating Solver Folder');
end
mkdir(folder_name);

% read track .json OR .yaml file
if strcmp(track_file{2}, 'json')
    track_data = readJSON([track_path, track_file{1}, '.json']);
    if order_cones == 1
        track_data = orderCones(track_data);
    end
elseif strcmp(track_file{2}, 'yaml')
    track_data = readYAML([track_path, track_file{1}, '.yaml']);
    track_data = orderCones(track_data);
%     track_data.X_b(end) = [];
%     track_data.Y_b(end) = [];
%     track_data.X_y(end) = [];
%     track_data.Y_y(end) = [];
%     track_data.X = flip(track_data.X);
%     track_data.Y = flip(track_data.Y);    % Plot the track
end

%% ----------   Generate Centerline Splines  -------------------------------

global track_length;

centerline.X = [track_data.X];
centerline.Y = [track_data.Y];
track_length = 0;
progress_list = zeros(1,length(centerline.X) + 1 + EXTEND_CENTERLINE*10); % Supposes that the centerline samples 10 points every meter but can sample less
full_centerline.X = zeros(1,length(centerline.X) + 1 + EXTEND_CENTERLINE*10);
full_centerline.Y = zeros(1,length(centerline.Y) + 1 + EXTEND_CENTERLINE*10);

% Calculate track_length and progress for each position
for i=1:length(centerline.X) - 1
    full_centerline.X(i) = centerline.X(i);
    full_centerline.Y(i) = centerline.Y(i);
    track_length = track_length + sqrt((centerline.X(i+1)-centerline.X(i))^2+(centerline.Y(i+1)-centerline.Y(i))^2);
    progress_list(i+1) = track_length;
end
full_centerline.X(i+1) = centerline.X(i+1);
full_centerline.Y(i+1) = centerline.Y(i+1);

% Last point to the starting point
i = i+2;
track_length = track_length + sqrt((centerline.X(1)-centerline.X(end))^2+(centerline.Y(1)-centerline.Y(end))^2);
progress_list(i) = track_length;
full_centerline.X(i) = centerline.X(1);
full_centerline.Y(i) = centerline.Y(1);

extended_length = 0;
j = 1;
while (extended_length<EXTEND_CENTERLINE)
    i = i + 1;
    full_centerline.X(i) = centerline.X(j);
    full_centerline.Y(i) = centerline.Y(j);
    extended_length = extended_length + sqrt((centerline.X(j+1)-centerline.X(j))^2+(centerline.Y(j+1)-centerline.Y(j))^2);
    progress_list(i) = track_length + extended_length;
    j = j + 1;
end
progress_list = progress_list(1:find(progress_list,1,'last'));
full_centerline.X = full_centerline.X(1:find(full_centerline.X,1,'last'));
full_centerline.Y = full_centerline.Y(1:find(full_centerline.Y,1,'last'));

% Downsample centerline points if needed
if DOWNSAMPLE_RATE
    new_centerline.X = zeros(1,floor(length(progress_list)/DOWNSAMPLE_RATE)+1);
    new_centerline.Y = zeros(1,floor(length(progress_list)/DOWNSAMPLE_RATE)+1);
    new_progress_list = zeros(1,floor(length(progress_list)/DOWNSAMPLE_RATE)+1);
    j = 1;
    for i=1:floor(length(progress_list)/DOWNSAMPLE_RATE)
        new_centerline.X(i) = full_centerline.X(i*DOWNSAMPLE_RATE-(DOWNSAMPLE_RATE-1));
        new_centerline.Y(i) = full_centerline.Y(i*DOWNSAMPLE_RATE-(DOWNSAMPLE_RATE-1));
        new_progress_list(i) = progress_list(i*DOWNSAMPLE_RATE-(DOWNSAMPLE_RATE-1));
    end
     new_centerline.X(end) = full_centerline.X(end);
    new_centerline.Y(end) = full_centerline.Y(end);
    new_progress_list(end) = progress_list(end);
else
    new_centerline.X = zeros(1,length(progress_list));
    new_centerline.Y = zeros(1,length(progress_list));
    new_progress_list = zeros(1,length(progress_list));
    j = 1;
    for i=1:floor(length(progress_list))
        if progress_list(i) < track_length
            new_centerline.X(i) = centerline.X(i);
            new_centerline.Y(i) = centerline.Y(i);
            new_progress_list(i) = progress_list(i);
        else
            new_centerline.X(i) = centerline.X(j);
            new_centerline.Y(i) = centerline.Y(j);
            new_progress_list(i) = progress_list(i);
            j = j + 1;
        end
    end
end

% Matlab Splines and Derivatives
matSpline.xSpline = spline(new_progress_list,new_centerline.X);
matSpline.ySpline = spline(new_progress_list,new_centerline.Y);
matSpline.dxSpline = fnder(matSpline.xSpline);
matSpline.dySpline = fnder(matSpline.ySpline);

%% ----------   Generate Track Splines       -------------------------------
n_points = 1000;

% Yellow
[resampled_yellow_points,~,resampled_yellow_func] = interparc(linspace(0,1,n_points),track_data.X_y,track_data.Y_y);
[] = distance2curve([[track_data.X;track_data.X(1)] [track_data.Y;track_data.Y(1)]],resampled_yellow_points,'spline');

% Blue

%% Plots 

figure(1)
plot(track_data.X, track_data.Y,'Color',"#000000")
hold on
plot(yellow_point_x,yellow_point_y,'Color',	"#EDB120")
% plot(blue_point_x,blue_point_y,'Color',"#0072BD")

% scatter(track_data.X_b, track_data.Y_b, 'blue', 'filled')
scatter(track_data.X_y, track_data.Y_y, 'yellow', 'filled')

spline_progress = 1;
point_progress = [fnval(matSpline.xSpline,progress),fnval(matSpline.ySpline,progress)];

[point_close,~,frac_progress] = distance2curve([[track_data.X_y;track_data.X_y(1)]  [track_data.Y_y;track_data.Y_y(1)] ],point_progress,'spline')

scatter(fnval(matSpline.xYellowCones,frac_progress*yellow_arc_length))
% Adicionar o extended

% Falta encontrar o progresso mais proximo quer para os cones amarelos como
% para os azuis do ponto inicial da centerline (e possivelmente o último
% também)

% Depois fazer um resample Que comece aí

% Fazer outro resample em que se faz uma regra de tres simples para ver
% qual é que corresponde a qual (possivelmente conseguem ser feitos no
% mesmo resample)