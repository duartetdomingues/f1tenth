function track_length = trackLengthFunc(track)
%% Gen spline
N = length(track.X) + 1;

pp_x = spline(1:N, [track.X(1:end); track.X(1)]); % WHY NOT JUST ADD THE LAST VALUE OF X AS WELL?
pp_y = spline(1:N, [track.Y(1:end); track.Y(1)]);

% normalize to arc length:
cumul_dist = zeros(N, 1);
for i = 2:N
    cumul_dist(i) = cumul_dist(i-1) + integral(@(s) fun(s, pp_x.coefs(i-1,:), pp_y.coefs(i-1,:), pp_x.breaks(i-1)), pp_x.breaks(i-1), pp_x.breaks(i),'RelTol',1e-48);
end

track_length = cumul_dist(end);

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