function cost = costFunc(s_mesh, track_length)

N_fine_nodes = 1000;

[g_x, g_y] = track_full(s_mesh);

pp_x = spline(s_mesh, g_x);
pp_y = spline(s_mesh, g_y);

fine_s_mesh = linspace(0, track_length, N_fine_nodes);

g_x_red = ppval(pp_x, fine_s_mesh);
g_y_red = ppval(pp_y, fine_s_mesh);

[g_x, g_y] = track_full(fine_s_mesh);

sq_err = NaN(N_fine_nodes, 1);
for i = 1:N_fine_nodes
    sq_err(i) = sqrt((g_x(i) - g_x_red(i))^2 + (g_y(i) - g_y_red(i))^2);
end

sq_err = exp(sq_err);

cost = mean(sq_err);