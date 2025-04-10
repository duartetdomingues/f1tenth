function trackSplineGenerator(val, nodes_per_meter, cost_threshold)
disp('Creating optimized track function...')
%% Parameters

if nodes_per_meter < 0
    nodes_per_meter = 35/309; % number of piecewise polinomials to use in
end

%% Gen spline
N = length(val.X) ;

pp_x = spline(1:N, [val.X(1:end-1); val.X(1)]);
pp_y = spline(1:N, [val.Y(1:end-1); val.Y(1)]);

% normalize to arc length:
cumul_dist = zeros(N, 1);
for i = 2:N
    cumul_dist(i) = cumul_dist(i-1) + integral(@(s) fun(s, pp_x.coefs(i-1,:), pp_y.coefs(i-1,:), pp_x.breaks(i-1)), pp_x.breaks(i-1), pp_x.breaks(i),'RelTol',1e-48);
end

track_length = cumul_dist(end);

N_intervals = round(track_length*nodes_per_meter);

theta = atan2((val.Y(2) - val.Y(end-1)),(val.X(2) - val.Y(end-1)));

s_mesh = [cumul_dist(1) - 0.5; cumul_dist; cumul_dist(end) + 0.5];
x_mesh = [val.X(1) - 0.5*cos(theta); val.X(1:end-1); val.X(1); val.X(1) + 0.5*cos(theta)];
y_mesh = [val.Y(1) - 0.5*sin(theta); val.Y(1:end-1); val.Y(1); val.Y(1) + 0.5*sin(theta)];
% fine mesh
pp_x = spline(s_mesh, x_mesh);
pp_y = spline(s_mesh, y_mesh);

pp_x.breaks = pp_x.breaks(2:(end-1));
pp_x.coefs = pp_x.coefs(2:end-1,:);
pp_x.pieces = size(pp_x.coefs, 1);

pp_y.breaks = pp_y.breaks(2:(end-1));
pp_y.coefs = pp_y.coefs(2:end-1,:);
pp_y.pieces = size(pp_y.coefs, 1);

% coarse mesh
% disp(['Using ', num2str(N_intervals), ' nodes'])
[s_mesh_old, cost] = genMesh(N_intervals, track_length, []);

for i = 1:10
    if cost < cost_threshold
        N_intervals = N_intervals - 1;
        s_mesh_old = s_mesh;
        disp(['Trying ', num2str(N_intervals), ' nodes'])
        [s_mesh, cost] = genMesh(N_intervals, track_length, []);
    else
        s_mesh = s_mesh_old;
        N_intervals = length(s_mesh);
        break;
    end
end

disp(['Performing final optimization with ', num2str(N_intervals), ' nodes...']);

s_mesh = unique(genMesh(N_intervals, track_length, s_mesh));
s_mesh = s_mesh(:);
N_intervals = length(s_mesh);

disp(['Generating final track function with ', num2str(N_intervals), ' nodes...']);

s_mesh_optimal = s_mesh;

g_x = ppval(pp_x, s_mesh);
g_y = ppval(pp_y, s_mesh);

s_mesh = [s_mesh(1) - 0.5; s_mesh; s_mesh(end) + 0.5];
x_mesh = [g_x(1) - 0.5*cos(theta); g_x(1:end); g_x(end) + 0.5*cos(theta)];
y_mesh = [g_y(1) - 0.5*sin(theta); g_y(1:end); g_y(end) + 0.5*sin(theta)];

pp_x = spline(s_mesh, x_mesh);
pp_y = spline(s_mesh, y_mesh);

pp_x.breaks = pp_x.breaks(2:(end-1));
pp_x.coefs = pp_x.coefs(2:end-1,:);
pp_x.pieces = size(pp_x.coefs, 1);

pp_y.breaks = pp_y.breaks(2:(end-1));
pp_y.coefs = pp_y.coefs(2:end-1,:);
pp_y.pieces = size(pp_y.coefs, 1);


g_x = ppval(pp_x, 0:0.01:cumul_dist(end));
g_y = ppval(pp_y, 0:0.01:cumul_dist(end));
plot(g_x, g_y)

%% Get symbolic expressions
syms s

s = s - floor(s/cumul_dist(end))*cumul_dist(end);

g_x = 0; g_y = 0;
for i = 1:pp_x.pieces
    % X
    form = heaviside(s - pp_x.breaks(i)) - heaviside(s - pp_x.breaks(i+1));
    a = pp_x.coefs(i,1);
    b = pp_x.coefs(i,2);
    c = pp_x.coefs(i,3);
    d = pp_x.coefs(i,4);
    x1 = pp_x.breaks(i);
    g_x = g_x + form*( a*(s - x1)^3 + b*(s - x1)^2 + c*(s - x1) + d );
    
    % Y
    form = heaviside(s - pp_y.breaks(i)) - heaviside(s - pp_y.breaks(i+1));
    a = pp_y.coefs(i,1);
    b = pp_y.coefs(i,2);
    c = pp_y.coefs(i,3);
    d = pp_y.coefs(i,4);
    x1 = pp_y.breaks(i);
    g_y = g_y + form*( a*(s - x1)^3 + b*(s - x1)^2 + c*(s - x1) + d );
    
end

%% derivatives
dg_x = 0; dg_y = 0;
for i = 1:pp_x.pieces
    % X
    form = heaviside(s - pp_x.breaks(i)) - heaviside(s - pp_x.breaks(i+1));
    a = pp_x.coefs(i,1);
    b = pp_x.coefs(i,2);
    c = pp_x.coefs(i,3);
    %d = pp_x.coefs(i,4);
    x1 = pp_x.breaks(i);
    dg_x = dg_x + form*( 3*a*(s - x1)^2 + 2*b*(s - x1) + c);
    
    % Y
    form = heaviside(s - pp_y.breaks(i)) - heaviside(s - pp_y.breaks(i+1));
    a = pp_y.coefs(i,1);
    b = pp_y.coefs(i,2);
    c = pp_y.coefs(i,3);
    %d = pp_y.coefs(i,4);
    x1 = pp_y.breaks(i);
    dg_y = dg_y + form*( 3*a*(s - x1)^2 + 2*b*(s - x1) + c);
end
%Phi = atan2(dg_y, dg_x);

disp('Simplify g_x')
g_x = simplify(g_x);
disp('Simplify g_y')
g_y = simplify(g_y);
disp('Simplify dg_x')
dg_x = simplify(dg_x);
disp('Simplify dg_y')
dg_y = simplify(dg_y);
%% Create a matlab function
matlabFunction(g_x, g_y, dg_x, dg_y, 'File', 'track');

%% 2st derivatives
d2g_x = 0; d2g_y = 0;
for i = 1:pp_x.pieces
    % X
    form = heaviside(s - pp_x.breaks(i)) - heaviside(s - pp_x.breaks(i+1));
    a = pp_x.coefs(i,1);
    b = pp_x.coefs(i,2);
    %c = pp_x.coefs(i,3);
    %d = pp_x.coefs(i,4);
    x1 = pp_x.breaks(i);
    d2g_x = d2g_x + form*(6*a*(s - x1) + 2*b);
    
    % Y
    form = heaviside(s - pp_y.breaks(i)) - heaviside(s - pp_y.breaks(i+1));
    a = pp_y.coefs(i,1);
    b = pp_y.coefs(i,2);
    %c = pp_y.coefs(i,3);
    %d = pp_y.coefs(i,4);
    x1 = pp_y.breaks(i);
    d2g_y = d2g_y + form*(6*a*(s - x1) + 2*b);
end
matlabFunction(g_x, g_y, dg_x, dg_y, d2g_x, d2g_y, 'File', 'track_2derivatives');

%% Test track function
figure('Name', 'A track', 'Position', [10 10, 600, 600])

s = 0.001:0.01:310-0.001;
[g_x, g_y] = track(s);

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

% q = quiver(g_x(1:1000:end), g_y(1:1000:end), dg_x(1:1000:end), dg_y(1:1000:end), 'color', 'k');
% %q.AutoScaleFactor = 0.4;
% legend('Right limit', 'Left limit', 'Center line', 'Center line dir.');


legend([left, right, start_line, start_dir, cline], {'Left limit', 'Right limit', 'Start cones', 'Travel dir.', 'Center line'});
end


%% -------------- function definitions ---------------
function dg = fun(s, c_x, c_y, s0)
x1 = s0;
y1 = s0;
a = c_x(1);
b = c_x(2);
c = c_x(3);
dg_x = 3*a*(s-x1).^2 + 2*b*(s-x1) + c;
a = c_y(1);
b = c_y(2);
c = c_y(3);
dg_y = 3*a*(s-y1).^2 + 2*b*(s-y1) + c;

dg = sqrt(dg_x.^2 + dg_y.^2);
end