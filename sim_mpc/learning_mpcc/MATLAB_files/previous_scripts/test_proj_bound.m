close all 

global xSpline_FORCES;
global ySpline_FORCES;
global dxSpline_FORCES;
global dySpline_FORCES;
global distYellow_FORCES;
global distBlue_FORCES;
xBlueSpline = matSpline.xBlueSpline;
yBlueSpline = matSpline.yBlueSpline;

s = 45;

width = 1.465;
length = 2.912;

[~, ~, dg_x, dg_y] = splineTrack(s);
temp = sqrt(dg_x^2 + dg_y^2);
dg_x = dg_x/temp;
dg_y = dg_y/temp;
dg = [dg_x; dg_y];

% x(1) = xSpline_FORCES(s)+dg_y;
% x(2) = ySpline_FORCES(s)-dg_x;
x(1) = xSpline_FORCES(s)-dg_y;
x(2) = ySpline_FORCES(s)+dg_x;
x(3) = -0.95;

yaw = x(3);

safety_margin = 0.25;
l_f = 0.88;
l_r = 0.66;

front_vector = [cos(x(3));sin(x(3))]*(l_f/(l_f + l_r))*length;
rear_vector = [-cos(x(3));-sin(x(3))]*(l_r/(l_f + l_r))*length;

pos = [x(1);x(2)];
front_pos = [x(1); x(2)] + front_vector;
rear_pos = [x(1); x(2)] + rear_vector;

front_proj = front_vector'*dg;
rear_proj = rear_vector'*dg;

[front_dist_blue,front_dist_yellow] = distanceBoundaries(s+front_proj);
[rear_dist_blue,rear_dist_yellow] = distanceBoundaries(s+rear_proj);

%Front
[gf_x, gf_y, dgf_x, dgf_y] = splineTrack(s+front_proj);
front_CL_pos = [gf_x;gf_y];
temp = sqrt(dgf_x^2 + dgf_y^2);
dgf_x = dgf_x/temp;
dgf_y = dgf_y/temp;
dgf = [dgf_x; dgf_y];
% dgf_orthog = [-dgf_y; dgf_x];
e_F = front_pos - front_CL_pos;
s_compensation_front =  dgf'*e_F;

[gf_x, gf_y, dgf_x, dgf_y] = splineTrack(s+front_proj+s_compensation_front);
front_CL_pos = [gf_x;gf_y];
temp = sqrt(dgf_x^2 + dgf_y^2);
dgf_x = dgf_x/temp;
dgf_y = dgf_y/temp;
dgf = [dgf_x; dgf_y];
dgf_orthog = [-dgf_y; dgf_x];
e_F = front_pos - front_CL_pos;
e_F_CL = dgf_orthog'*e_F;

% Rear
[gr_x_prev, gr_y_prev, dgr_x, dgr_y] = splineTrack(s+rear_proj);
rear_CL_pos = [gr_x_prev;gr_y_prev];
temp = sqrt(dgr_x^2 + dgr_y^2);
dgr_x = dgr_x/temp;
dgr_y = dgr_y/temp;
dgr = [dgr_x; dgr_y];
% dgr_orthog = [-dgr_y; dgr_x];
e_R = rear_pos - rear_CL_pos;
s_compensation_rear = dgr'*e_R;

[gr_x, gr_y, dgr_x, dgr_y] = splineTrack(s+rear_proj+s_compensation_rear);
rear_CL_pos = [gr_x;gr_y];
temp = sqrt(dgr_x^2 + dgr_y^2);
dgr_x = dgr_x/temp;
dgr_y = dgr_y/temp;
dgr_orthog = [-dgr_y; dgr_x];

e_R = rear_pos - rear_CL_pos;
e_R_CL = dgr_orthog'*e_R;

% Center
[g_x, g_y, dg_x, dg_y] = splineTrack(s);
CL_pos = [g_x;g_y];
temp = sqrt(dg_x^2 + dg_y^2);
dg_x = dgr_x/temp;
dg_y = dgr_y/temp;
dg_orthog = [-dg_y; dg_x];

e_C = pos - CL_pos;
e_C_CL = dg_orthog'*e_C;

for i=1:21
    path(i,1) = xSpline_FORCES(s-5+((i-1)/20)*10);
    path(i,2) = ySpline_FORCES(s-5+((i-1)/20)*10);
    yellow(i,1) = fnval(matSpline.xYellowSpline,s-5+((i-1)/20)*10);
    yellow(i,2) = fnval(matSpline.yYellowSpline,s-5+((i-1)/20)*10);
    blue(i,1) = fnval(matSpline.xBlueSpline,s-5+((i-1)/20)*10);
    blue(i,2) = fnval(matSpline.yBlueSpline,s-5+((i-1)/20)*10);
end

car_center_x = x(1);
car_center_y = x(2);

corner_front_left_x = car_center_x+(l_f/(l_f + l_r))*length;
corner_front_left_y = car_center_y+width/2;

corner_front_right_x = car_center_x+(l_f/(l_f + l_r))*length;
corner_front_right_y = car_center_y-width/2;

corner_rear_left_x = car_center_x-(l_r/(l_f + l_r))*length;
corner_rear_left_y = car_center_y+width/2;

corner_rear_right_x = car_center_x-(l_r/(l_f + l_r))*length;
corner_rear_right_y = car_center_y-width/2;

R = [cos(x(3)) -sin(x(3)); sin(x(3)) cos(x(3))];
% R = eye(2);

corner_front_left = R*[corner_front_left_x-car_center_x; corner_front_left_y-car_center_y];
corner_front_right = R*[corner_front_right_x-car_center_x; corner_front_right_y-car_center_y];
corner_rear_left = R*[corner_rear_left_x-car_center_x; corner_rear_left_y-car_center_y];
corner_rear_right =R*[corner_rear_right_x-car_center_x; corner_rear_right_y-car_center_y];

corner_front_left = corner_front_left + [car_center_x;car_center_y];
corner_front_right =  corner_front_right + [car_center_x;car_center_y];
corner_rear_left = corner_rear_left  + [car_center_x;car_center_y];
corner_rear_right = corner_rear_right + [car_center_x;car_center_y];

corners = [corner_front_left corner_front_right corner_rear_right corner_rear_left corner_front_left];

plot(corners(1,:), corners(2,:));
axis equal
hold on
plot(path(:,1),path(:,2),'Color',"#77AC30");
plot(yellow(:,1),yellow(:,2),'Color',"#EDB120")
plot(blue(:,1),blue(:,2),'Color',"#0072BD")
scatter(front_pos(1),front_pos(2));
scatter(rear_pos(1),rear_pos(2));
scatter(xSpline_FORCES(s),ySpline_FORCES(s))
scatter(x(1),x(2))
scatter(gr_x,gr_y)
scatter(gr_x_prev,gr_y_prev)
scatter(gf_x,gf_y)
P2 = [front_pos(1) front_pos(2);
      rear_pos(1) rear_pos(2)
      pos(1)    pos(2)];
P1 = [gf_x gf_y;
      gr_x gr_y
      g_x g_y];
arrow3(P1,P2,'k',0.25,0.25,0,1,0.4);
scatter(fnval(xBlueSpline,s+rear_proj+s_compensation_rear),fnval(yBlueSpline,s+rear_proj+s_compensation_rear))
scatter(fnval(xBlueSpline,s+rear_proj),fnval(yBlueSpline,s+rear_proj))

[distance_yellow_front, distance_blue_front] = distanceBoundaries(s+front_proj+s_compensation_front);
[distance_yellow_rear, distance_blue_rear] = distanceBoundaries(s+rear_proj+s_compensation_rear);
[distance_yellow, distance_blue] = distanceBoundaries(s);


title_text = sprintf(['Error in Front: %.2f, Max allowed RAW: [%.2f; -%.2f], Max allowed: [%.2f; -%.2f]' ...
    '\nError in Center: %.2f, Max allowed RAW: [%.2f; -%.2f], Max allowed: [%.2f; -%.2f]' ...
    '\nError in Rear: %.2f, Max allowed RAW: [%.2f; -%.2f], Max allowed: [%.2f; -%.2f]' ...
    '\n Center Left Condition: %.2f\n Center Right Condition: %.2f\n Front Left Condition: %.2f\n Front Right Condition: %.2f\n Rear Left Condition: %.2f\n Rear Right Condition: %.2f' ...
    '\n Left < 0 ; Right > 0'], ...
    e_F_CL,distance_blue_front,distance_yellow_front,distance_blue_front-width/2-safety_margin,distance_yellow_front-width/2-safety_margin, ...
    e_C_CL,distance_blue,distance_yellow,distance_blue-width/2-safety_margin,distance_yellow-width/2-safety_margin, ...
    e_R_CL,distance_blue_rear,distance_yellow_rear,distance_blue_rear-width/2-safety_margin,distance_yellow_rear-width/2-safety_margin, ...
    (e_C_CL)-(distance_blue-width/2-safety_margin), ...
    (e_C_CL)+(distance_yellow-width/2-safety_margin), ...
    (e_F_CL)-(distance_blue_front-width/2-safety_margin), ...
    (e_F_CL)+(distance_yellow_front-width/2-safety_margin), ...
    (e_R_CL)-(distance_blue_rear-width/2-safety_margin), ...
    (e_R_CL)+(distance_yellow_rear-width/2-safety_margin));

title(title_text);

function [x,y,dx,dy] = splineTrack(s)
    global xSpline_FORCES;
    global ySpline_FORCES;
    global dxSpline_FORCES;
    global dySpline_FORCES;

    x = xSpline_FORCES(s);
    y = ySpline_FORCES(s);
    dx = dxSpline_FORCES(s);
    dy = dySpline_FORCES(s);
end

function [distance_yellow, distance_blue] = distanceBoundaries(s)
    global distYellow_FORCES;
    global distBlue_FORCES;

    distance_yellow = distYellow_FORCES(s);
    distance_blue = distBlue_FORCES(s);
end

%ADICIONAR A OUTRA COMPONETE DO ERRO PARA COMPENSAR O MISMATCH DA PROJEÇÃO
%DE PROGRESSO, COMO SE ADICIONASSE O ERRO DE LAG PARA COMPENSAR