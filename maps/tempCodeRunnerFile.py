x_s = spline_x(s_uniform)
y_s = spline_y(s_uniform)
dx = spline_x.derivative(1)(s_uniform)
dy = spline_y.derivative(1)(s_uniform)
ddx = spline_x.derivative(2)(s_uniform)
ddy = spline_y.derivative(2)(s_uniform)
kappa = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
