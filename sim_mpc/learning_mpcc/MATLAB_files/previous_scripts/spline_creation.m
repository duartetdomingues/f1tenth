function [matlab,centerline_arc_length,cones,centerline,folder_name]  = spline_creation(spline_points,extended_points)

    % Spline Paramenters
    SPLINE_POINTS = spline_points;
    EXTENDED_POINTS = extended_points; % Each point is seperated by centerline_arc_length/SPLINE_POINTS of distance, 0 currently not supported
    
    %% ----------   Select track file   -------------------------------------
    
    [track_file,track_path] = uigetfile('Tracks/*.json;*.sdf;*.yaml', 'Select a track file');
    % track_file = 'acceleration.yaml';
    % track_file = 'acceleration_2_points.yaml';
    % track_file = 'trackdrive.yaml';
    % track_file = 'estoril_13_03_third_try_again.yaml'
    % track_path = '/home/david/fst/autonomous-systems/src/control/learning_mpcc/MATLAB_files/Tracks/';
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
%         track_data = orderCones(track_data);
    %     track_data.X_b(end) = [];
    %     track_data.Y_b(end) = [];
    %     track_data.X_y(end) = [];
    %     track_data.Y_y(end) = [];
%         track_data.X = flip(track_data.X);
%         track_data.Y = flip(track_data.Y);    % Plot the track
    end
    
    %% ----------   CENTERLINE                   -------------------------------
    
%     centerline.X = [track_data.X; track_data.X(1)];
%     centerline.Y = [track_data.Y; track_data.Y(1)];
    cones.blue = [track_data.X_b track_data.Y_b];
    cones.yellow = [track_data.X_y track_data.Y_y];
    centerline.X = [track_data.X];
    centerline.Y = [track_data.Y];
    
    centerline_arc_length = 0;
    centerline_progress = zeros(length(centerline.X),1);
    for i=1:length(centerline.X)-1
        centerline_arc_length = centerline_arc_length + sqrt((centerline.X(i+1)-centerline.X(i))^2+(centerline.Y(i+1)-centerline.Y(i))^2);
        centerline_progress(i+1) = centerline_arc_length;
    end
    
    spline_centerline_x = spline(centerline_progress,centerline.X);
    spline_centerline_y = spline(centerline_progress,centerline.Y);
    
    resampled_centerline_progress = zeros(SPLINE_POINTS,1);
    resampled_centerline_x = zeros(SPLINE_POINTS,1);
    resampled_centerline_y = zeros(SPLINE_POINTS,1);
    
    for i=1:SPLINE_POINTS
        resampled_centerline_progress(i) = centerline_arc_length*(i-1)/(SPLINE_POINTS-1);
        resampled_centerline_x(i) = fnval(spline_centerline_x,resampled_centerline_progress(i));
        resampled_centerline_y(i) = fnval(spline_centerline_y,resampled_centerline_progress(i));
    end
    
    matlab.xSpline = spline(resampled_centerline_progress,resampled_centerline_x);
    matlab.ySpline = spline(resampled_centerline_progress,resampled_centerline_y);
    
    %% ----------   Generate Track Splines       -------------------------------
    
    % Yellow
    [resampled_yellow_points,~,~] = interparc(linspace(0,0.9,SPLINE_POINTS),track_data.X_y,track_data.Y_y);
    [~,~,frac_progress] = distance2curve([centerline.X centerline.Y],resampled_yellow_points,'spline');
    [yellow_progress,index] = sort(frac_progress);
    yellow_progress = yellow_progress * centerline_arc_length;
    ordered_yellow_points = resampled_yellow_points(index,:);
    
    matlab.xYellowSpline = spline(yellow_progress,ordered_yellow_points(:,1));
    matlab.yYellowSpline = spline(yellow_progress,ordered_yellow_points(:,2));
    
    matlab.dxYellowSpline = fnder(matlab.xYellowSpline);
    matlab.dyYellowSpline = fnder(matlab.yYellowSpline);
    
    % Blue
    [resampled_blue_points,~,~] = interparc(linspace(0,0.9,SPLINE_POINTS),track_data.X_b,track_data.Y_b);
    [~,~,frac_progress] = distance2curve([centerline.X centerline.Y],resampled_blue_points,'spline');
    [blue_progress,index] = sort(frac_progress);
    blue_progress = blue_progress * centerline_arc_length;
    ordered_blue_points = resampled_blue_points(index,:);
    
    matlab.xBlueSpline = spline(blue_progress,ordered_blue_points(:,1));
    matlab.yBlueSpline = spline(blue_progress,ordered_blue_points(:,2));
    
    matlab.dxBlueSpline = fnder(matlab.xBlueSpline);
    matlab.dyBlueSpline = fnder(matlab.yBlueSpline);
    
    % figure
    % hold on
    % plot(resampled_centerline_x, resampled_centerline_y,'Color',"#77AC30")
    % plot(resampled_yellow_points(:,1),resampled_yellow_points(:,2),'Color',	"#EDB120");
    % scatter(track_data.X_y, track_data.Y_y, 10,[0.9290 0.6940 0.1250], 'filled')
    % plot(resampled_blue_points(:,1),resampled_blue_points(:,2),'Color',		"#0072BD");
    % scatter(track_data.X_b, track_data.Y_b, 10,[0 0.4470 0.7410], 'filled')
    % axis equal
    
    % scatter(matched_points(9,1), matched_points(9,2))
    % scatter(resampled_yellow_points(9,1), resampled_yellow_points(9,2))
    % scatter(fnval(matlab.xSpline,frac_progress(9)*centerline_arc_length),fnval(matlab.ySpline,frac_progress(9)*centerline_arc_length))
    % scatter(matched_points(end,1),matched_points(end,2))
    % scatter(resampled_yellow_points(end,1),resampled_yellow_points(end,2))
    
    % for i=1:1001
    %     x(i) = fnval(matlab.xSpline,centerline_arc_length*(i-1)/1000);
    %     y(i) = fnval(matlab.ySpline,centerline_arc_length*(i-1)/1000);
    %     xy(i) = fnval(matlab.xYellowSpline,centerline_arc_length*(i-1)/1000);
    %     yy(i) = fnval(matlab.yYellowSpline,centerline_arc_length*(i-1)/1000);
    %     xb(i) = fnval(matlab.xBlueSpline,centerline_arc_length*(i-1)/1000);
    %     yb(i) = fnval(matlab.yBlueSpline,centerline_arc_length*(i-1)/1000);
    % end
    % 
    % plot(x,y)
    % hold on
    % plot(xy,yy)
    % plot(xb,yb)
    
    %% ----------   EXTEND SPLINES               -------------------------------
    
    if EXTENDED_POINTS
        dist_per_point = centerline_arc_length/SPLINE_POINTS;
        extra_progress = centerline_arc_length+dist_per_point:dist_per_point:centerline_arc_length+dist_per_point*EXTENDED_POINTS;
        extra_progress = extra_progress';
        equivalent_progress = dist_per_point:dist_per_point:dist_per_point*EXTENDED_POINTS;
    %     equivalent_progress = equivalent_progress';
    
        extended_centerline_progress = [resampled_centerline_progress; extra_progress];
        extended_yellow_progress = [yellow_progress; extra_progress];
        extended_blue_progress = [blue_progress; extra_progress];
    
        extended_centerline = zeros(EXTENDED_POINTS,2);
        extended_yellow_points = zeros(EXTENDED_POINTS,2);
        extended_blue_points = zeros(EXTENDED_POINTS,2);
    
        for i=1:EXTENDED_POINTS
            extended_centerline(i,1) = fnval(matlab.xSpline,equivalent_progress(i));
            extended_centerline(i,2) = fnval(matlab.ySpline,equivalent_progress(i));
            extended_yellow_points(i,1) = fnval(matlab.xYellowSpline,equivalent_progress(i));
            extended_yellow_points(i,2) = fnval(matlab.yYellowSpline,equivalent_progress(i));
            extended_blue_points(i,1) = fnval(matlab.xBlueSpline,equivalent_progress(i));
            extended_blue_points(i,2) = fnval(matlab.yBlueSpline,equivalent_progress(i));
        end
    
        extended_centerline = [resampled_centerline_x resampled_centerline_y; extended_centerline];
        extended_blue_points = [ordered_blue_points; extended_blue_points];
        extended_yellow_points = [ordered_yellow_points; extended_yellow_points];
    
        % Splines
        matlab.xSpline = spline(extended_centerline_progress,extended_centerline(:,1));
        matlab.ySpline = spline(extended_centerline_progress,extended_centerline(:,2));
    
        matlab.xYellowSpline = spline(extended_yellow_progress,extended_yellow_points(:,1));
        matlab.yYellowSpline = spline(extended_yellow_progress,extended_yellow_points(:,2));
    
        matlab.xBlueSpline = spline(extended_blue_progress,extended_blue_points(:,1));
        matlab.yBlueSpline = spline(extended_blue_progress,extended_blue_points(:,2));
    
        % Derivatives
        matlab.dxSpline = fnder(matlab.xSpline);
        matlab.dySpline = fnder(matlab.ySpline);
    
%         matlab.dxYellowSpline = fnder(matlab.xYellowSpline);
%         matlab.dyYellowSpline = fnder(matlab.yYellowSpline);
    
%         matlab.dxBlueSpline = fnder(matlab.xBlueSpline);
%         matlab.dyBlueSpline = fnder(matlab.yBlueSpline);
    
    end
    
    %% ----------   DISTANCE SPLINES             -------------------------------
    x = zeros(SPLINE_POINTS+EXTENDED_POINTS,1);
    y = zeros(SPLINE_POINTS+EXTENDED_POINTS,1);
    xy = zeros(SPLINE_POINTS+EXTENDED_POINTS,1);
    yy = zeros(SPLINE_POINTS+EXTENDED_POINTS,1);
    xb = zeros(SPLINE_POINTS+EXTENDED_POINTS,1);
    yb = zeros(SPLINE_POINTS+EXTENDED_POINTS,1);
    distance_yellow = zeros(SPLINE_POINTS+EXTENDED_POINTS,1);
    distance_blue = zeros(SPLINE_POINTS+EXTENDED_POINTS,1);
    
    for i=1:SPLINE_POINTS+EXTENDED_POINTS
        x(i) = fnval(matlab.xSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS-1));
        y(i) = fnval(matlab.ySpline,centerline_arc_length*(i-1)/(SPLINE_POINTS-1));
        xy(i) = fnval(matlab.xYellowSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS-1));
        yy(i) = fnval(matlab.yYellowSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS-1));
        xb(i) = fnval(matlab.xBlueSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS-1));
        yb(i) = fnval(matlab.yBlueSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS-1));
    
        distance_yellow(i) = sqrt((x(i)-xy(i))^2 + (y(i)-yy(i))^2);
        distance_blue(i) = sqrt((x(i)-xb(i))^2 + (y(i)-yb(i))^2);
    end
    
    matlab.distanceYellowSpline = spline(extended_yellow_progress,distance_yellow);
    matlab.distanceBlueSpline = spline(extended_yellow_progress,distance_blue);
    
    %% ----------   PLOTS                        -------------------------------
    UPSAMPLE = 3.0;

    for i=1:(SPLINE_POINTS+EXTENDED_POINTS)*UPSAMPLE
        x(i) = fnval(matlab.xSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS*UPSAMPLE-1));
        y(i) = fnval(matlab.ySpline,centerline_arc_length*(i-1)/(SPLINE_POINTS*UPSAMPLE-1));
        xy(i) = fnval(matlab.xYellowSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS*UPSAMPLE-1));
        yy(i) = fnval(matlab.yYellowSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS*UPSAMPLE-1));
        xb(i) = fnval(matlab.xBlueSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS*UPSAMPLE-1));
        yb(i) = fnval(matlab.yBlueSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS*UPSAMPLE-1));
        distance_yellow(i) = fnval(matlab.distanceYellowSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS*UPSAMPLE-1));
        distance_blue(i) = fnval(matlab.distanceBlueSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS*UPSAMPLE-1));
    
    %     plot(track_data.X,track_data.Y,'--','Color','black');
    %     hold on
    %     scatter(track_data.X_y, track_data.Y_y, 10,[0.9290 0.6940 0.1250], 'filled')
    %     scatter(track_data.X_b, track_data.Y_b, 10,[0 0.4470 0.7410], 'filled')
    %     plot(x(1:i),y(1:i),'Color',"#77AC30");
    %     plot(xy(1:i),yy(1:i),'Color',"#EDB120");
    %     plot(xb(1:i),yb(1:i),'Color',"#0072BD");
    %     title_text = sprintf('Dist Yellow: %0.3f, Dist Blue: %0.3f',distance_yellow(i), distance_blue(i));
    %     title(title_text)
    %     axis equal
    %     drawnow 
    %     pause(0.005)
    %     w = waitforbuttonpress;
    end
    
    plot(track_data.X,track_data.Y,'--','Color','black');
    hold on
    scatter(track_data.X_y, track_data.Y_y, 10,[0.9290 0.6940 0.1250], 'filled')
    scatter(track_data.X_b, track_data.Y_b, 10,[0 0.4470 0.7410], 'filled')
    plot(x(1:i),y(1:i),'Color',"#77AC30");
    plot(xy(1:i),yy(1:i),'Color',"#EDB120");
    plot(xb(1:i),yb(1:i),'Color',"#0072BD");
    x = fnval(matlab.xSpline,linspace(0,centerline_arc_length,30));
    y = fnval(matlab.ySpline,linspace(0,centerline_arc_length,30));
    P1 = [x' y'];
    dx = fnval(matlab.dxSpline,linspace(0,centerline_arc_length,30));
    dy = fnval(matlab.dySpline,linspace(0,centerline_arc_length,30));
    P2 = [(x+2*dx)' (y+2*dy)'];
    axis equal
    arrow3(P1,P2,'k',0.25,0.25,0,1,0.4);
    hold off


