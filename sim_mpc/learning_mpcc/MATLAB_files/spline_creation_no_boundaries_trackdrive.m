function [matlab,centerline_arc_length,cones,centerline,folder_name]  = spline_creation_no_boundaries_trackdrive(spline_points,extended_points)

    % Spline Paramenters
    SPLINE_POINTS = spline_points;
    EXTENDED_POINTS = extended_points; % Each point is seperated by centerline_arc_length/SPLINE_POINTS of distance, 0 currently not supported
    
    %% ----------   Select track file   -------------------------------------
    
%     [track_file,track_path] = uigetfile('Tracks/*.json;*.sdf;*.yaml', 'Select a track file');
    track_file = 'trackdrive.yaml';
    track_path = '../../../estimation/graphslam/maps/';

    track_file = split(track_file, '.');
    order_cones = 1;
    
    % Read in case of sdf
    if strcmp(track_file{2}, 'sdf')
        readSDF([track_path, track_file{1}], false);
        order_cones = 0;
    end
    
    % create output directory
    folder_name = ['output_', track_file{1}, '_', 'MPCC'];
    
    % read track .json OR .yaml file
    if strcmp(track_file{2}, 'json')
        track_data = readJSON([track_path, track_file{1}, '.json']);
        if order_cones == 1
            track_data = orderCones(track_data);
        end
    elseif strcmp(track_file{2}, 'yaml')
        track_data = readYAML([track_path, track_file{1}, '.yaml']);
    end
    
    %% ----------   CENTERLINE                   -------------------------------
    
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
    
    %% ----------   EXTEND SPLINES               -------------------------------
    
    if EXTENDED_POINTS
        dist_per_point = centerline_arc_length/SPLINE_POINTS;
        extra_progress = centerline_arc_length+dist_per_point:dist_per_point:centerline_arc_length+dist_per_point*EXTENDED_POINTS;
        extra_progress = extra_progress';
        equivalent_progress = dist_per_point:dist_per_point:dist_per_point*EXTENDED_POINTS;
    %     equivalent_progress = equivalent_progress';
    
        extended_centerline_progress = [resampled_centerline_progress; extra_progress];
    
        extended_centerline = zeros(EXTENDED_POINTS,2);
    
        for i=1:EXTENDED_POINTS
            extended_centerline(i,1) = fnval(matlab.xSpline,equivalent_progress(i));
            extended_centerline(i,2) = fnval(matlab.ySpline,equivalent_progress(i));
        end
    
        extended_centerline = [resampled_centerline_x resampled_centerline_y; extended_centerline];
    
        % Splines
        matlab.xSpline = spline(extended_centerline_progress,extended_centerline(:,1));
        matlab.ySpline = spline(extended_centerline_progress,extended_centerline(:,2));
    
        % Derivatives
        matlab.dxSpline = fnder(matlab.xSpline);
        matlab.dySpline = fnder(matlab.ySpline);
    
    end
    
    %% ----------   PLOTS                        -------------------------------
    UPSAMPLE = 5.0;

    for i=1:(SPLINE_POINTS+EXTENDED_POINTS)*UPSAMPLE
        x(i) = fnval(matlab.xSpline,centerline_arc_length*(i-1)/(SPLINE_POINTS*UPSAMPLE-1));
        y(i) = fnval(matlab.ySpline,centerline_arc_length*(i-1)/(SPLINE_POINTS*UPSAMPLE-1));
    end
    
    plot(track_data.X,track_data.Y,'--','Color',[.25 .25 .25]);
    hold on
    scatter(track_data.X_y, track_data.Y_y, 10,[0.9290 0.6940 0.1250], 'filled')
    scatter(track_data.X_b, track_data.Y_b, 10,[0 0.4470 0.7410], 'filled')
    plot(x(1:i),y(1:i),'Color',"#77AC30");
    x = fnval(matlab.xSpline,linspace(0,centerline_arc_length,30));
    y = fnval(matlab.ySpline,linspace(0,centerline_arc_length,30));
    P1 = [x' y'];
    dx = fnval(matlab.dxSpline,linspace(0,centerline_arc_length,30));
    dy = fnval(matlab.dySpline,linspace(0,centerline_arc_length,30));
    P2 = [(x+2*dx)' (y+2*dy)'];
    axis equal
    arrow3(P1,P2,'k',0.25,0.25,0,1,0.4);
    hold off


