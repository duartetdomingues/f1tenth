float64[] x_real # current measured states [x, y, yaw, vx, vy, yaw_rate]
float64[] x_blended # states predicted by the mpc model [x, y, yaw, vx, vy, yaw_rate]
float64[] diff # error between the real and predicted values (x_real - x_blended)
