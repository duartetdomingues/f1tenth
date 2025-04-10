load('André_2000-06-23_06-00-28.mat')

time = netas_EKF_x_13_vx.time(1:end-1);
time_prev = netas_EKF_x_13_vx.time(2:end);

time_diff = time-time_prev;

plot(time_diff)