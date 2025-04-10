function model = ocp_model_acados()

    import casadi.*

    x = SX.sym('x', 5);  % States: [x, y, psi, theta, v]
    u = SX.sym('u', 2);  % Controls: [delta, v]
    L = 0.35;

    % System dynamics
    dx = [ ...
        x(5)*cos(x(3));        % dx/dt
        x(5)*sin(x(3));        % dy/dt
        (x(5)/L)*tan(u(1));    % dpsi/dt
        0;
        0];                    % dtheta, dv (modelled as direct input)

    % Formulate model
    model.x = x;
    model.u = u;
    model.f_expl_expr = dx;

end
