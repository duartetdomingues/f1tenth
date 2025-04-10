function [track_length,xSpline,ySpline,dxSpline,dySpline] = splineGenerator(track_data)

    centerline.X = [track_data.X; track_data.X(1)];
    centerline.Y = [track_data.Y; track_data.Y(1)];

    track_length = 0;
    progress_list = zeros(1,length(centerline.X));
    for i=1:length(centerline.X)-1
        track_length = track_length + sqrt((centerline.X(i+1)-centerline.X(i))^2+(centerline.Y(i+1)-centerline.Y(i))^2);
        progress_list(i+1) = track_length;
    end
%     [track_length, progress_list] = calculate_track_length(centerline.X,centerline.Y);
    
    xSpline = spline(progress_list,centerline.X);
    ySpline = spline(progress_list,centerline.Y);
    dxSpline = fnder(xSpline);
    dySpline = fnder(ySpline);
    
    %% Plot data
    figure
    % centerline
%     plot(centerline.X,centerline.Y,'--');
    hold on
    % spline
    plot(ppval(xSpline,0:track_length),ppval(ySpline,0:track_length));
    % spline derivative
    x = ppval(xSpline,linspace(0,track_length,25));
    y = ppval(ySpline,linspace(0,track_length,25));
    P1 = [x' y'];
    dx = ppval(dxSpline,linspace(0,track_length,25));
    dy = ppval(dySpline,linspace(0,track_length,25));
    P2 = [(x+2*dx)' (y+2*dy)'];
%     arrow3(P1,P2,'k',0.25,0.25,0,1,0.4);
    hold off
    
    %% COnversion for ForcesPro

    xSpline_FORCES = ForcesInterpolation(xSpline);
    ySpline_FORCES = ForcesInterpolation(ySpline);
    dxSpline_FORCES = ForcesInterpolation(dxSpline);
    dySpline_FORCES = ForcesInterpolation(dySpline);
        
    save("track.mat","xSpline_FORCES","ySpline_FORCES","dxSpline_FORCES","dySpline_FORCES");
end