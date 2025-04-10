import os
import shutil
import yaml
import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
from scipy.integrate import solve_ivp
from tkinter import Tk     # from tkinter import Tk for Python 3.x
from tkinter.filedialog import askopenfilename
from MPCModelFuncs import differential_mpc_model
from MPCModelFuncs import differential_mpc_model_no_load, differential_mpc_model_data_diff, differential_mpc_model_data_diff_and_load, planar_model_TV
from MPCModelFuncs import euler_step
import matplotlib.pyplot as plt
import random

## Script Parameters
bag_folder = '/home/david/bags/pre_as_bootcamp/to_be_trained/'
dynamic_model = 4 # 0: No load trasnfer, 1: Load transfer, 2: Data Differential (no load), 3 : Data Differential (withload), 4 : Planar
T = 1 # Number of previous timesteps to use as input
h = 1/20 # Sampling time
train_test_split = 0.75 # Percentage of data to use for training
ammount_to_keep = 0.95
add_inverse = False
shuffle = True
need_to_validate = False
##

approval = False
def on_key(event):
    global approval
    if event.key == 'x':
        print("Data denied!")
        plt.close()  # Close the plot window
        approval = False
    elif event.key == 'enter':
        print("Data approved!")
        plt.close()  # Close the plot window
        approval = True
    else:
        print("Other key is pressed")


def plot_and_validate_data(vx,vy,r,throttle,steering):
    fig, ax = plt.subplots(5, 1, figsize=(100, 100))

    ax[0].plot(vx,linewidth=1)
    ax[0].set_xlabel('Timestep',fontsize=10)
    ax[0].set_ylabel('Velocity x [m/s]',fontsize=10)

    ax[1].plot(vy,linewidth=1)
    ax[1].set_xlabel('Timestep',fontsize=10)
    ax[1].set_ylabel('Velocity y [m/s]',fontsize=10)

    ax[2].plot(r,linewidth=1)
    ax[2].set_xlabel('Timestep',fontsize=10)
    ax[2].set_ylabel('Yaw Rate [rad/s]',fontsize=10)
    
    ax[3].plot(throttle,linewidth=1)
    ax[3].set_xlabel('Timestep',fontsize=10)
    ax[3].set_ylabel('Throttle',fontsize=10)
    
    ax[4].plot(steering,linewidth=1)
    ax[4].set_xlabel('Timestep',fontsize=10)
    ax[4].set_ylabel('Steering [rad]',fontsize=10)
    
    fig.canvas.mpl_connect('key_press_event', on_key)

    plt.show()

# Important Script Parameters
folder_name = os.path.basename(bag_folder[:-1])

home_directory = os.path.expanduser("~")
final_output_name = home_directory + '/fst/autonomous-systems/src/control/learning_mpcc/AI/train_data/' + folder_name + '_T' + str(T) + '_model_' + str(dynamic_model)
if shuffle == True:
    final_output_name = final_output_name + '_shuffled'
else:
    final_output_name = final_output_name + '_not_shuffled'
    
try:
    os.mkdir(final_output_name)
except:
    shutil.rmtree(final_output_name)
    os.mkdir(final_output_name)

rejected_output_name = bag_folder + 'rejected'
try:
    os.mkdir(rejected_output_name)
except:
    print('Rejected folder already exists')


# Read yaml file
p = np.zeros(24)
with open(home_directory + '/fst/autonomous-systems/src/control/learning_mpcc/config/default.yaml') as stream:
    parameters = yaml.load(stream, Loader=yaml.SafeLoader)
    p[0] = parameters['model_params']['l_f']
    p[1] = parameters['model_params']['l_r']
    p[2] = parameters['model_params']['m']
    p[3] = parameters['model_params']['I_z']
    p[4] = parameters['model_params']['T_max_front']
    p[5] = parameters['model_params']['T_max_rear']
    p[6] = parameters['model_params']['T_brake_front']
    p[7] = parameters['model_params']['T_brake_rear']
    p[8] = parameters['model_params']['GR']
    p[9] = parameters['model_params']['eta_motor']
    p[10] = parameters['model_params']['r_wheel']
    p[11] = parameters['model_params']['g']
    p[12] = parameters['model_params']['C_roll']
    p[13] = parameters['model_params']['rho']
    p[14] = parameters['model_params']['C_d']
    p[15] = parameters['model_params']['C_l']
    p[16] = parameters['tyre_params']['B']
    p[17] = parameters['tyre_params']['C']
    p[18] = parameters['tyre_params']['D']
    p[19] = parameters['model_params']['downforce_front']
    p[20] = parameters['model_params']['downforce_rear']
    p[21] = parameters['model_params']['h_cog']
    p[22] = parameters['model_params']['track_width']
    p[23] = parameters['model_params']['diff_gain']
    vel_min = parameters['model_params']['lambda_blend_min']
        

bags = [f for f in os.listdir(bag_folder) if f.endswith(".bag")]
output_name = []
decrease_j = 0

for j,bag_name in enumerate(bags):
    j = j - decrease_j
    
    try:
        # Read bag file
        b = bagreader(bag_folder + bag_name)    
        NN_DATA_MSG = b.message_by_topic('/control/learning_mpcc/nn_data')
        nn_data = pd.read_csv(NN_DATA_MSG)
    except:
        print('No NN data in bag file. Skipping ...')
        decrease_j = decrease_j + 1
        continue
    
    output_name.append(home_directory+'/fst/autonomous-systems/src/control/learning_mpcc/AI/train_data' + b.filename[:-4] + '_T' + str(T) + '.npz')

    # Current control action
    
    throttles = np.array(nn_data['control_cmd.throttle'])
    steerings = np.array(nn_data['control_cmd.steering_angle'])
    # Previous control action
    prev_throttles = np.zeros((T,throttles.size))
    prev_steerings = np.zeros((T,steerings.size))
    for i in range(T):
        prev_throttles[i,:] = np.roll(throttles,i+1)
        prev_steerings[i,:] = np.roll(steerings,i+1)

    # Remove first and last element of current control action
    throttles = throttles[T:-T]
    steerings = steerings[T:-T]
    # Remove first and last element of previous control action
    prev_throttles = prev_throttles[:,T:-T]
    prev_steerings = prev_steerings[:,T:-T]

    # Current State
    vx = np.array(nn_data['car_velocity.velocity.x'])
    vy = np.array(nn_data['car_velocity.velocity.y'])
    r = np.array(nn_data['car_velocity.velocity.theta'])
    # Next States (Current State shifted by one timestep)
    vx_next = np.roll(vx, -1)
    vy_next = np.roll(vy, -1)
    r_next = np.roll(r, -1) 
    # Previous States (Current State shifted by one timestep)
    vx_prev = np.zeros((T,vx.size))
    vy_prev = np.zeros((T,vy.size))
    r_prev = np.zeros((T,r.size))
    for i in range(T):
        vx_prev[i,:] = np.roll(vx,i+1)
        vy_prev[i,:] = np.roll(vy,i+1)
        r_prev[i,:] = np.roll(r,i+1)

    # Remove first and last element of current states
    vx = vx[T:-T]
    vy = vy[T:-T]
    r = r[T:-T]
    # Remove first and last element of next states
    vx_next = vx_next[T:-T]
    vy_next = vy_next[T:-T]
    r_next = r_next[T:-T]
    # Remove first and last element of previous states
    vx_prev = vx_prev[:,T:-T]
    vy_prev = vy_prev[:,T:-T]
    r_prev = r_prev[:,T:-T]

    ## Remove Velocity under a certain threshold
    vel = np.sqrt(np.power(vx,2)+np.power(vy,2))
    # Remove elements with velocity lower than the min
    # Current control action
    throttles = throttles[vel>vel_min]
    steerings = steerings[vel>vel_min]
    # Previous control action
    prev_throttles = prev_throttles[:,vel>vel_min]
    prev_steerings = prev_steerings[:,vel>vel_min]
    # Current States
    vx = vx[vel>vel_min]
    vy = vy[vel>vel_min]
    r = r[vel>vel_min]
    # Previous States
    vx_prev = vx_prev[:,vel>vel_min]
    vy_prev = vy_prev[:,vel>vel_min]
    r_prev = r_prev[:,vel>vel_min]
    # Next States
    vx_next = vx_next[vel>vel_min]
    vy_next = vy_next[vel>vel_min]
    r_next = r_next[vel>vel_min]
    
    ## Remove a percentage of the end (where Manel makes the car stop with VSV brake but then makes in run again)
    # Current control action
    size = int(len(throttles)*ammount_to_keep)
    throttles = throttles[:size]
    steerings = steerings[:size]
    # Previous control action
    prev_throttles = prev_throttles[:,:size]
    prev_steerings = prev_steerings[:,:size]
    # Current States
    vx = vx[:size]
    vy = vy[:size]
    r = r[:size]
    # Previous States
    vx_prev = vx_prev[:,:size]
    vy_prev = vy_prev[:,:size]
    r_prev = r_prev[:,:size]
    # Next States
    vx_next = vx_next[:size]
    vy_next = vy_next[:size]
    r_next = r_next[:size]
    
    ## Remove the last control inputs where brake is requested
    delete = 0
    for i in range(len(throttles) - 1, -1, -1):
        if throttles[i] == throttles[i-1]:
            delete = delete + 1
        elif delete > 0:
            delete = delete + 1
            break
        else:
            break
    
    if delete > 0:
        # Current control action
        throttles = throttles[:-delete]
        steerings = steerings[:-delete]
        # Previous control action
        prev_throttles = prev_throttles[:,:-delete]
        prev_steerings = prev_steerings[:,:-delete]
        # Current States
        vx = vx[:-delete]
        vy = vy[:-delete]
        r = r[:-delete]
        # Previous States
        vx_prev = vx_prev[:,:-delete]
        vy_prev = vy_prev[:,:-delete]
        r_prev = r_prev[:,:-delete]
        # Next States
        vx_next = vx_next[:-delete]
        vy_next = vy_next[:-delete]
        r_next = r_next[:-delete]
    
    # Validate data through user
    if need_to_validate:
        plot_and_validate_data(vx,vy,r,throttles,steerings)
        if not approval:
            shutil.move(bag_folder + bag_name, rejected_output_name)
            continue

    # Add opposite control and states
    if add_inverse == True:
        vx_inv = vx
        vx_prev_inv = vx_prev
        vy_inv = -vy
        vy_prev_inv = -vy_prev
        r_inv = -r
        r_prev_inv = -r_prev
        throttles_inv = throttles
        prev_throttles_inv = prev_throttles
        steerings_inv = -steerings
        prev_steerings_inv = -prev_steerings
        vx_next_inv = vx_next
        vy_next_inv = -vy_next
        r_next_inv = -r_next

        vx = np.concatenate([vx, vx_inv],axis=0)
        vx_prev = np.concatenate([vx_prev, vx_prev_inv],axis=1)
        vy = np.concatenate([vy, vy_inv],axis=0)
        vy_prev = np.concatenate([vy_prev, vy_prev_inv],axis=1)
        r = np.concatenate([r, r_inv],axis=0)
        r_prev = np.concatenate([r_prev, r_prev_inv],axis=1)
        throttles = np.concatenate([throttles, throttles_inv],axis=0)
        prev_throttles = np.concatenate([prev_throttles, prev_throttles_inv],axis=1)
        steerings = np.concatenate([steerings, steerings_inv],axis=0)
        prev_steerings = np.concatenate([prev_steerings, prev_steerings_inv],axis=1)
        vx_next = np.concatenate([vx_next, vx_next_inv],axis=0)
        vy_next = np.concatenate([vy_next, vy_next_inv],axis=0)
        r_next = np.concatenate([r_next, r_next_inv],axis=0)

        
        
    # Apply current control action to model and get error
    states = np.array([vx, vy, r]).T
    states_next = np.array([vx_next, vy_next, r_next]).T
    if dynamic_model == 0:
        pred_dynamics = differential_mpc_model_no_load(states, np.array([throttles, steerings, prev_throttles[0,:], prev_steerings[0,:]]).T,p)
    elif dynamic_model == 1:
        pred_dynamics = differential_mpc_model(np.array([vx, vy, r]).T, np.array([throttles, steerings, prev_throttles[0,:], prev_steerings[0,:]]).T,p)
    elif dynamic_model == 2:
        pred_dynamics = differential_mpc_model_data_diff(np.array([vx, vy, r]).T, np.array([throttles, steerings, prev_throttles[0,:], prev_steerings[0,:]]).T,p)
    elif dynamic_model == 3:
        pred_dynamics = differential_mpc_model_data_diff_and_load(np.array([vx, vy, r]).T, np.array([throttles, steerings, prev_throttles[0,:], prev_steerings[0,:]]).T,p)
    elif dynamic_model == 4:
        pred_dynamics = planar_model_TV(np.array([vx, vy, r, vx_prev[0,:], vy_prev[0,:]]).T, np.array([throttles, steerings, prev_throttles[0,:], prev_steerings[0,:]]).T,p,h)
    else:
        print('No valid dynamic model selected')
        exit()
    
    pred_states = euler_step(np.array([vx, vy, r]),pred_dynamics,h)
    error = states_next.T - pred_states

    x = np.zeros((throttles.size,T+1,5))
    x[:,-1,:] = np.array([throttles, steerings, vx, vy, r]).T
    for i in range(T):
        x[:,-(i+2),:] = np.array([prev_throttles[i,:], prev_steerings[i,:], vx_prev[i,:], vy_prev[i,:], r_prev[i,:]]).T
    
    y = (error/h).T
    
    idxs = [i for i in range(1, len(x))]
    if shuffle:
        random.shuffle(idxs)
    
    suffled_x = x[idxs]
    suffled_y = y[idxs]
    shuffled_next = np.array([vx_next, vy_next, r_next]).T[idxs]

    x_train, x_test = suffled_x[0:int(len(x)*train_test_split)], suffled_x[int(len(x)*train_test_split):]
    y_train, y_test = suffled_y[0:int(len(y)*train_test_split)], suffled_y[int(len(y)*train_test_split):]
    states_test = x_test[:,:,2:]
    states_next_test = shuffled_next[int(len(x)*train_test_split):]

    print(output_name[j])

    np.savez(output_name[j],x_train=x_train,y_train=y_train,x_test=x_test,y_test=y_test,states=states,states_next=states_next,states_test=states_test,states_next_test=states_next_test)

for file in output_name:
    # Move the file to the destination folder
    try:
        shutil.move(file, final_output_name)
        print('File moved to ' + final_output_name)
    except:
        print('File rejected - Empty!')
        shutil.move(file, rejected_output_name)
        continue