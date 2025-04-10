function track_length = trackSplineGeneratorFunc(name, val)
%% Gen spline
N = length(val.X);

pp_x = spline(1:N, [val.X(1:end-1); val.X(1)]); % WHY NOT JUST ADD THE LAST VALUE OF X AS WELL?
pp_y = spline(1:N, [val.Y(1:end-1); val.Y(1)]);

% normalize to arc length:
cumul_dist = zeros(N, 1);
for i = 2:N
    cumul_dist(i) = cumul_dist(i-1) + integral(@(s) fun(s, pp_x.coefs(i-1,:), pp_y.coefs(i-1,:), pp_x.breaks(i-1)), pp_x.breaks(i-1), pp_x.breaks(i),'RelTol',1e-48);
end

track_length = cumul_dist(end);

N_intervals = round(60/300*track_length); disp(['Creating dense track function with ', num2str(N_intervals), ' nodes.']);

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
s_mesh = linspace(cumul_dist(1), cumul_dist(end), N_intervals).';

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

%% Create a matlab function
matlabFunction(g_x, g_y, dg_x, dg_y, 'File', name);

end
%% -------------- function definition ---------------
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