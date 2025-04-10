function path = input_scenario_V3(path_tag)

switch path_tag    
    case '27_02_23'
        load('paths/27_02_23.mat','pathArray');
        path = pathArray;
        path(:,6) = 0;
    case '13_04_21'
        load('paths/13_04_21.mat','path');
        path(:,6) = 0;
    case '06_05_21'
        load('paths/06_05_21.mat','path');
        path(:,6) = 0;
    case '11_05_21'
        load('paths/11_05_21.mat','path');
        path(:,6) = 0;
    case 'lin_ret'
        load('paths/lin_ret.mat', 'path');
    case '05_05_21'
        load('paths/05_05_21.mat','path');

    case 'circ'
        s = 0:0.2:100;
        xy = [100*cos(s/100*2*pi); 100*sin(s/100*2*pi)];
        path = path_def(xy);
        path = [path(1,50:end-1)' path(2,50:end-1)' path(3,50:end-1)' path(4,1:end-50)' path(5,1:end-50)'];
        
    case 'sin'
        s = 0:0.2:100;
        xy = [s; 100*sin(s/100*2*pi)];
        path = path_def(xy);
        path = [path(1,1:end-1)' path(2,1:end-1)' path(3,1:end-1)' path(4,1:end-1)'];
    
    case 'spiral'
        t = 1:0.2:400;
        u = .0265;
        r0 = 10;
        r = r0 +u*t;
        omega = 1.2;
        phi0 = 3*pi/2;
        phi = -omega*t+phi0;
        x = r .* cos(phi);
        y = r .* sin(phi);
        path = path_def([x; y]);
        path = [path(1,1:end-1)' path(2,1:end-1)' path(3,1:end-1)' path(4,1:end-1)'];
        plot(path(:,1), path(:,2))
        
    case 'spiral2'
        
        t = 1:0.2:100;
        u = 0.1;
        r0 = 10;
        r = r0 +u*t;
        omega = 0.1;
        phi0 = 3*pi/2;
        phi = -omega*t+phi0;
        x = r .* cos(phi);
        y = r .* sin(phi);
        path = path_def([x; y]);
        path = [path(1,1:end-1)' path(2,1:end-1)' path(3,1:end-1)' path(4,1:end-1)'];
        plot(path(:,1), path(:,2))
        
    case 'spiral3'
        
        t = 1:0.2:55;
        u = 0.0265;
        r0 = 10;
        r = r0 +u*t;
        omega = 0.2;
        phi0 = 3*pi/2;
        phi = -omega*t+phi0;
        x = r .* cos(phi);
        y = r .* sin(phi);
        path = path_def([x; y]);
        path = [path(1,1:end-1)' path(2,1:end-1)' path(3,1:end-1)' path(4,1:end-1)'];
        plot(path(:,1), path(:,2))
        
    case 'spiral4'
        t = 1:0.2:200;
        u = .15;
        r0 = 2;
        r = r0 +u*t;
        omega = 1;
        phi0 = 3*pi/2;
        phi = -omega*t+phi0;
        x = r .* cos(phi);
        y = r .* sin(phi);
        path = path_def([x; y]);
        path = [path(1,1:end-1)' path(2,1:end-1)' path(3,1:end-1)' path(4,1:end-1)'];
        path = flip(path);
        figure(101)
        plot(path(:,1), path(:,2))
        hold on
        scatter(path(1,1), path(1,2))
        
    case 'line'
        x = 1:0.2:100;
        y = 1:0.2:100;
        path = path_def([x; y]);
        path = [path(1,1:end-1)' path(2,1:end-1)' path(3,1:end-1)' path(4,1:end-1)'];
        
    case 'long_line'
        x = 1:0.2:500;
        y(1:length(x)) = 5;
        path = path_def([x; y]);
        path = [path(1,1:end-1)' path(2,1:end-1)' path(3,1:end-1)' path(4,1:end-1)'];
        
    case 'line2'
        x = 1:0.2:100;
        y(1:length(x)) = 5;
        path = path_def([x; y]);
        path = [path(1,1:end-1)' path(2,1:end-1)' path(3,1:end-1)' path(4,1:end-1)'];

    case 'elbow'
        x1 = 1:0.2:50;
        y1(1:length(x1)) = 5;
        y2 = 5:0.2:55;
        x2(1:length(y2)) = 50;
        x = [x1 x2];
        y = [y1 y2];
        path = path_def([x; y]);
        path = [path(1,1:end-1)' path(2,1:end-1)' path(3,1:end-1)' path(4,1:end-1)'];
      
end

end