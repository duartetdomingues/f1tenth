function [x,y,psi ,X_s, Y_s,theta_s]=local_to_global_pose(track, model_var,x)

    s_val=x(1);
    n_val = x(2);
    mu_val= x(3);
    
    pathlength = track.s_traj(end);
    s_val = rem(s_val, pathlength);

    X_s = interp1(track.s_traj, track.x_traj, s_val, 'linear', 'extrap');
    Y_s = interp1(track.s_traj, track.y_traj, s_val, 'linear', 'extrap');
    theta_traj = cumtrapz(track.s_traj, track.kappa_traj);  % integração de kappa para obter theta

    theta_0 = atan2(track.y_traj(2)-track.y_traj(1),track.x_traj(2)-track.x_traj(1));
    theta_traj= theta_traj+theta_0;
    theta_s = interp1(track.s_traj, theta_traj, s_val, 'linear', 'extrap');
    

    x = X_s + n_val * cos(theta_s + pi/2);
    y = Y_s + n_val * sin(theta_s + pi/2);
    psi = theta_s + mu_val;
end