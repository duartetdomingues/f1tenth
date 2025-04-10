# %% Initialization

import yaml
import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
from scipy.integrate import solve_ivp
from tkinter import Tk     # from tkinter import Tk for Python 3.x
from tkinter.filedialog import askopenfilename

# Important Script Parameters

# Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
filename = askopenfilename() # show an "Open" dialog box and return the path to the selected file
print(filename)
bag_name = '/home/david/bags/estoril-13-03-2023/trackdrive_raw_2023-03-13-18-59-42.bag'
T = 1 # Number of previous timesteps to use as input
h = 0.050 # Sampling time


# Read bag file
b = bagreader(bag_name)
output_name = 'train_data' + b.filename

# Read yaml file
p = np.zeros(22)
with open('/home/david/fst/autonomous-systems/src/control/learning_mpcc/config/default.yaml') as stream:
    parameters = yaml.load(stream, Loader=yaml.SafeLoader)
    p[0] = parameters['model_params']['l_f'];
    p[1] = parameters['model_params']['l_r'];
    p[2] = parameters['model_params']['m'];
    p[3] = parameters['model_params']['I_z'];
    p[4] = parameters['model_params']['T_max_front'];
    p[5] = parameters['model_params']['T_max_rear'];
    p[6] = parameters['model_params']['T_brake_front'];
    p[7] = parameters['model_params']['T_brake_rear'];
    p[8] = parameters['model_params']['GR'];
    p[9] = parameters['model_params']['eta_motor'];
    p[10] = parameters['model_params']['r_wheel'];
    p[11] = parameters['model_params']['g'];
    p[12] = parameters['model_params']['C_roll'];
    p[13] = parameters['model_params']['rho'];
    p[14] = parameters['model_params']['C_d'];
    p[15] = parameters['model_params']['C_l'];
    p[16] = parameters['tyre_params']['B'];
    p[17] = parameters['tyre_params']['C'];
    p[18] = parameters['tyre_params']['D'];
    p[19] = parameters['model_params']['downforce_front'];
    p[20] = parameters['model_params']['downforce_rear'];
    p[21] = parameters['model_params']['h_cog'];
    vel_min = parameters['model_params']['lambda_blend_min'];
    
# Dynamic model
def differential_mpc_model(x,u):
    ''' y = [X
             Y
             Psi
             vx
             vy
             r
             throttle
             steering
             throttle_prev
             steering_prev]'''
    
    # Define parameters
    l_f = p[0]
    l_r = p[1]
    m = p[2]
    I_z = p[3]
    T_max_front = p[4]
    T_max_rear = p[5]
    T_brake_front = p[6]
    T_brake_rear = p[7]
    GR = p[8]
    eta_motor = p[9]
    r_wheel = p[10]
    g = p[11]
    C_roll = p[12]
    rho = p[13]
    C_d = p[14]
    C_l = p[15]
    B = p[16]
    C = p[17]
    D = p[18]
    downforce_front = p[19]
    downforce_rear = p[20]
    h_cog = p[21]
    
    # Define states and inputs vectors
    # x = np.array([y(1), y(2), y(3), y(4), y(5), y(6)])
    # u = np.array([y(7), y(8), y(9), y(10)])
    
    # Braking or Accel Torque
    t_bool = (u[:,0] < 0)
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front)
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear)

    # Rolling Resistance
    F_roll = C_roll*m*g
    
    # Drag Force
    F_drag = 1/2*rho*C_d*np.power(x[:,3],2)

    # Static weight distribution
    Ff_z_static = m * (l_r/(l_f + l_r)) * g
    Fr_z_static = m * (l_f/(l_f + l_r)) * g

    # Downforce Distribution
    F_downforce = 0.5 * rho * C_l * np.power(x[:,3],2)

    Ff_downforce = F_downforce * downforce_front
    Fr_downforce = F_downforce * downforce_rear

    # Slip Angles
    alpha_f = u[:,2] - np.arctan2(x[:,4] + x[:,5]*l_f, x[:,3])
    alpha_r = np.arctan2((x[:,4] - x[:,5]*l_r), x[:,3])

    # Longitudinal Load Transfer
    F_f_ax = 2*eta_motor*T_front*GR/r_wheel*u[:,2]
    F_r_ax = 2*eta_motor*T_rear*GR/r_wheel*u[:,2]
    Ff_tire_load = (Ff_z_static+Ff_downforce)*D*np.sin(C*np.arctan(B*alpha_f))
    a_x = (F_f_ax*np.cos(u[:,3]) - Ff_tire_load*np.sin(u[:,3]) + F_r_ax - F_roll - F_drag)/m
    delta_Fz = h_cog/(l_r+l_f)*m*a_x

    # Static + Downforce + Longitudinal Load Transfer
    Ff_z = Ff_z_static + Ff_downforce - delta_Fz
    Fr_z = Fr_z_static + Fr_downforce + delta_Fz

    # Lateral Forces
    Ff_tire = Ff_z*D*np.sin(C*np.arctan(B*alpha_f))
    Fr_tire = Fr_z*D*np.sin(C*np.arctan(B*alpha_r))
   
    # Torque Requests
    F_f = 2*eta_motor*T_front*GR/r_wheel*u[:,0]
    F_r = 2*eta_motor*T_rear*GR/r_wheel*u[:,0]
    
    # Longitudinal and Lateral Force and Yaw Moment
    F_x = F_f*np.cos(u[:,1]) - Ff_tire*np.sin(u[:,1]) + F_r - F_roll - F_drag
    F_y = F_f*np.sin(u[:,1]) + Ff_tire*np.cos(u[:,1]) - Fr_tire
    M_z = (F_f*np.sin(u[:,1]) + Ff_tire*np.cos(u[:,1]))*l_f + Fr_tire*l_r
    
    # State derivative function
    cos_Phi = np.cos(x[:,2])
    sin_Phi = np.sin(x[:,2])
    dX = np.array([x[:,3]*cos_Phi - x[:,4]*sin_Phi,\
                  x[:,3]*sin_Phi + x[:,4]*cos_Phi,\
                  x[:,5],\
                  F_x/m + x[:,4]*x[:,5],\
                  F_y/m - x[:,3]*x[:,5],\
                  M_z/I_z])
    
    return dX

def euler_step(x, dX, h):
    return x + h*dX


# %% Get topics

# Current control action
CONTROL_MSG = b.message_by_topic('/control/controller/control_cmd')
control_cmd = pd.read_csv(CONTROL_MSG)
throttles = np.array(control_cmd['throttle'])
steerings = np.array(control_cmd['steering_angle'])
# Previous control action
prev_throttles = np.roll(throttles,1)
prev_steerings = np.roll(steerings,1)

# Remove first and last element of current control action
throttles = throttles[1:-1]
steerings = steerings[1:-1]
# Remove first and last element of previous control action
prev_throttles = prev_throttles[1:-1]
prev_steerings = prev_steerings[1:-1]

# Current State
CURRENT_STATE_MSG = b.message_by_topic('/control/learning_mpcc/mpc_model')
states = pd.read_csv(CURRENT_STATE_MSG)
X = np.array(states['X_real.X'])
Y = np.array(states['X_real.Y'])
Psi = np.array(states['X_real.Psi'])
vx = np.array(states['X_real.vx'])
vy = np.array(states['X_real.vy'])
r = np.array(states['X_real.r'])
# Next States (Current State shifted by one timestep)
X_next = np.roll(X, -1)
Y_next = np.roll(Y, -1)
Psi_next = np.roll(Psi, -1)
vx_next = np.roll(vx, -1)
vy_next = np.roll(vy, -1)
r_next = np.roll(r, -1) 
# Previous States (Current State shifted by one timestep)
X_prev = np.roll(X, 1)
Y_prev = np.roll(Y, 1)
Psi_prev = np.roll(Psi, 1)
vx_prev = np.roll(vx, 1)
vy_prev = np.roll(vy, 1)
r_prev = np.roll(r, 1) 

# Difference between control inputs and staes
diff = len(X)-len(throttles)-2
# Remove first and last element of current states
X = X[diff+1:-1]
Y = Y[diff+1:-1]
Psi = Psi[diff+1:-1]
vx = vx[diff+1:-1]
vy = vy[diff+1:-1]
r = r[diff+1:-1]
# Remove first and last element of next states
X_next = X_next[diff+1:-1]
Y_next = Y_next[diff+1:-1]
Psi_next = Psi_next[diff+1:-1]
vx_next = vx_next[diff+1:-1]
vy_next = vy_next[diff+1:-1]
r_next = r_next[diff+1:-1]
# Remove first and last element of previous states
X_prev = X_prev[diff+1:-1]
Y_prev = Y_prev[diff+1:-1]
Psi_prev = Psi_prev[diff+1:-1]
vx_prev = vx_prev[diff+1:-1]
vy_prev = vy_prev[diff+1:-1]
r_prev = r_prev[diff+1:-1]

# Velocity
vel = np.sqrt(np.power(vx,2)+np.power(vy,2))
# Remove elements with velocity lower than the min
# 
throttles = throttles[vel>vel_min]
steerings = steerings[vel>vel_min]
# 
prev_throttles = prev_throttles[vel>vel_min]
prev_steerings = prev_steerings[vel>vel_min]
# 
X = X[vel>vel_min]
Y = Y[vel>vel_min]
Psi = Psi[vel>vel_min]
vx = vx[vel>vel_min]
vy = vy[vel>vel_min]
r = r[vel>vel_min]
# 
X_next = X_next[vel>vel_min]
Y_next = Y_next[vel>vel_min]
Psi_next = Psi_next[vel>vel_min]
vx_next = vx_next[vel>vel_min]
vy_next = vy_next[vel>vel_min]
r_next = r_next[vel>vel_min]
# 
X_prev = X_prev[vel>vel_min]
Y_prev = Y_prev[vel>vel_min]
Psi_prev = Psi_prev[vel>vel_min]
vx_prev = vx_prev[vel>vel_min]
vy_prev = vy_prev[vel>vel_min]
r_prev = r_prev[vel>vel_min]

# %% Data treatment

# Apply current control action to model and get error
pred_dynamics = differential_mpc_model(np.array([X, Y, Psi, vx, vy, r]).T, np.array([throttles, steerings, prev_throttles, prev_steerings]).T)
pred_states = euler_step(np.array([X, Y, Psi, vx, vy, r]),pred_dynamics,h)
error = pred_states - np.array([X_next, Y_next, Psi_next, vx_next, vy_next, r_next])

x = np.array([throttles, prev_throttles, steerings, prev_steerings, vx, vx_prev, vy, vy_prev, r, r_prev])
y = error[-3:]/h

# %% Write train data to file
np.savez(output_name,x=x,y=y)



# %%
