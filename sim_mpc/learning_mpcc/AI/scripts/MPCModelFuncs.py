import yaml
import numpy as np

def kinematic_mpc_model(x,u,p,h):
    '''x = [vx
        vy
        r]
        
    u = [throttle
            delta]'''
        
    
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
    
    # Sideslip angle
    Beta = np.arctan(l_r/(l_f+l_r)*np.tan(u[:,1]))
    
    # Braking or Accel Torque
    t_bool = (u[:,0] < 0)
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front)
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear)
    
    # Torque Requests
    F_f = 2*eta_motor*T_front*GR/r_wheel*u[:,0]
    F_r = 2*eta_motor*T_rear*GR/r_wheel*u[:,0]
    
    # Forces
    F = F_f + F_r
    
    dot_v = F/m
    r = x[:,0]*np.tan(u[:,1])/(l_f+l_r)
    
    X = np.array([x[:,0] + dot_v*np.cos(Beta),\
                  x[:,1] + dot_v*np.sin(Beta),\
                  r])
    
    return X    
    

def differential_mpc_model(x,u,p):
    '''x = [vx
        vy
        r]
        
    u = [throttle
            delta]'''
        
    
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
    
    # Braking or Accel Torque
    t_bool = (u[:,0] < 0)
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front)
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear)

    # Rolling Resistance
    F_roll = C_roll*m*g
    
    # Drag Force
    F_drag = 1/2*rho*C_d*np.power(x[:,0],2)

    # Static weight distribution
    Ff_z_static = m * (l_r/(l_f + l_r)) * g
    Fr_z_static = m * (l_f/(l_f + l_r)) * g

    # Downforce Distribution
    F_downforce = 0.5 * rho * C_l * np.power(x[:,0],2)

    Ff_downforce = F_downforce * downforce_front
    Fr_downforce = F_downforce * downforce_rear

    # Slip Angles
    alpha_f = u[:,1] - np.arctan2(x[:,1] + x[:,2]*l_f, x[:,0])
    alpha_r = np.arctan2((x[:,1] - x[:,2]*l_r), x[:,0])

    # Longitudinal Load Transfer
    F_f_ax = 2*eta_motor*T_front*GR/r_wheel*u[:,2]
    F_r_ax = 2*eta_motor*T_rear*GR/r_wheel*u[:,2]
    Ff_tire_load = (Ff_z_static+Ff_downforce)*D*np.sin(C*np.arctan(B*alpha_f))
    a_x = (F_f_ax*np.cos(u[:,3]) - Ff_tire_load*np.sin(u[:,3]) + F_r_ax - F_roll - F_drag)/m + x[:,1]*x[:,2]
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
    dX = np.array([F_x/m + x[:,1]*x[:,2],\
                  F_y/m - x[:,0]*x[:,2],\
                  M_z/I_z])
    
    return dX

def differential_mpc_model_no_load(x,u,p):
    '''x = [vx
        vy
        r]
        
    u = [throttle
            delta]'''
        
    
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
    
    # Braking or Accel Torque
    t_bool = (u[:,0] < 0)
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front)
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear)

    # Rolling Resistance
    F_roll = C_roll*m*g
    
    # Drag Force
    F_drag = 1/2*rho*C_d*np.power(x[:,0],2)

    # Static weight distribution
    Ff_z_static = m * (l_r/(l_f + l_r)) * g
    Fr_z_static = m * (l_f/(l_f + l_r)) * g

    # Downforce Distribution
    F_downforce = 0.5 * rho * C_l * np.power(x[:,0],2)

    Ff_downforce = F_downforce * downforce_front
    Fr_downforce = F_downforce * downforce_rear

    # Slip Angles
    alpha_f = u[:,1] - np.arctan2(x[:,1] + x[:,2]*l_f, x[:,0])
    alpha_r = np.arctan2((x[:,1] - x[:,2]*l_r), x[:,0])

    # Static + Downforce + Longitudinal Load Transfer
    Ff_z = Ff_z_static + Ff_downforce
    Fr_z = Fr_z_static + Fr_downforce

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
    dX = np.array([F_x/m + x[:,1]*x[:,2],\
                  F_y/m - x[:,0]*x[:,2],\
                  M_z/I_z])
    
    return dX

def differential_mpc_model_data_diff(x,u,p):
    '''x = [vx
        vy
        r]
        
    u = [throttle
            delta]'''
        
    
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
    track_width = p[22]
    diff_gain = p[23]
    
    # Braking or Accel Torque
    t_bool = (u[:,0] < 0)
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front)
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear)

    # Get differential torques
    diff_front, diff_rear = differential(x[:,0],u[:,1],diff_gain)
    T_fl = np.maximum(np.minimum(T_front + diff_front/2,T_max_front),-T_brake_front)
    T_fr = np.maximum(np.minimum(T_front - diff_front/2,T_max_front),-T_brake_front)
    T_rl = np.maximum(np.minimum(T_rear + diff_rear/2,T_max_rear),-T_brake_rear)
    T_rr = np.maximum(np.minimum(T_rear - diff_rear/2,T_max_rear),-T_brake_rear)
    F_fl = eta_motor*T_fl*GR/r_wheel*u[:,0]
    F_fr = eta_motor*T_fr*GR/r_wheel*u[:,0]
    F_rl = eta_motor*T_rl*GR/r_wheel*u[:,0]
    F_rr = eta_motor*T_rr*GR/r_wheel*u[:,0]
    diff_Mz = track_width/2*((F_fr-F_fl)+(F_rr-F_rl))

    # Rolling Resistance
    F_roll = C_roll*m*g
    
    # Drag Force
    F_drag = 1/2*rho*C_d*np.power(x[:,0],2)

    # Static weight distribution
    Ff_z_static = m * (l_r/(l_f + l_r)) * g
    Fr_z_static = m * (l_f/(l_f + l_r)) * g

    # Downforce Distribution
    F_downforce = 0.5 * rho * C_l * np.power(x[:,0],2)

    Ff_downforce = F_downforce * downforce_front
    Fr_downforce = F_downforce * downforce_rear

    # Slip Angles
    alpha_f = u[:,1] - np.arctan2(x[:,1] + x[:,2]*l_f, x[:,0])
    alpha_r = np.arctan2((x[:,1] - x[:,2]*l_r), x[:,0])

    # Static + Downforce + Longitudinal Load Transfer
    Ff_z = Ff_z_static + Ff_downforce
    Fr_z = Fr_z_static + Fr_downforce

    # Lateral Forces
    Ff_tire = Ff_z*D*np.sin(C*np.arctan(B*alpha_f))
    Fr_tire = Fr_z*D*np.sin(C*np.arctan(B*alpha_r))
   
    # Torque Requests
    F_f = F_fl + F_fr
    F_r = F_rl + F_rr
    
    # Longitudinal and Lateral Force and Yaw Moment
    F_x = F_f*np.cos(u[:,1]) - Ff_tire*np.sin(u[:,1]) + F_r - F_roll - F_drag
    F_y = F_f*np.sin(u[:,1]) + Ff_tire*np.cos(u[:,1]) - Fr_tire
    M_z = (F_f*np.sin(u[:,1]) + Ff_tire*np.cos(u[:,1]))*l_f + Fr_tire*l_r + diff_Mz
    
    # State derivative function
    dX = np.array([F_x/m + x[:,1]*x[:,2],\
                  F_y/m - x[:,0]*x[:,2],\
                  M_z/I_z])
    
    return dX

def dynamical_differential(x,u,p,h):
    '''x = [vx
        vy
        r]
        
    u = [throttle
            delta]'''
        
    
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
    track_width = p[22]
    diff_gain = p[23]


    # Rolling Resistance
    F_roll = C_roll*m*g
    
    # Drag Force
    F_drag = 1/2*rho*C_d*np.power(x[:,0],2)
    
    # Braking or Accel Torque
    t_bool = (u[:,0] < 0)
    T_front = ((~t_bool)*T_max_front + t_bool*(T_brake_front))*u[:,0]
    T_rear = ((~t_bool)*T_max_rear + t_bool*(T_brake_rear))*u[:,0]

    # Get differential torques
    a_x = get_ax(x,h)
    diff_front, diff_rear = differential(u[:,1],x[:,0],diff_gain,a_x)
    T_fl = np.maximum(np.minimum(T_front + diff_front/2,T_max_front),-T_brake_front)
    T_fr = np.maximum(np.minimum(T_front - diff_front/2,T_max_front),-T_brake_front)
    T_rl = np.maximum(np.minimum(T_rear + diff_rear/2,T_max_rear),-T_brake_rear)
    T_rr = np.maximum(np.minimum(T_rear - diff_rear/2,T_max_rear),-T_brake_rear)
    F_fl = eta_motor*T_fl*GR/r_wheel
    F_fr = eta_motor*T_fr*GR/r_wheel
    F_rl = eta_motor*T_rl*GR/r_wheel
    F_rr = eta_motor*T_rr*GR/r_wheel
    diff_Mz = track_width/2*((F_fr-F_fl)*np.cos(u[:,1])+(F_rr-F_rl))

    # Static weight distribution
    Ff_z_static = m * (l_r/(l_f + l_r)) * g
    Fr_z_static = m * (l_f/(l_f + l_r)) * g

    # Downforce Distribution
    F_downforce = 0.5 * rho * C_l * np.power(x[:,0],2)

    Ff_downforce = F_downforce * downforce_front
    Fr_downforce = F_downforce * downforce_rear

    # Slip Angles
    alpha_f = u[:,1] - np.arctan2(x[:,1] + x[:,2]*l_f, x[:,0])
    alpha_r = np.arctan2((x[:,1] - x[:,2]*l_r), x[:,0])

    # Static + Downforce + Longitudinal Load Transfer
    Ff_z = Ff_z_static + Ff_downforce
    Fr_z = Fr_z_static + Fr_downforce

    # Lateral Forces
    Ff_tire = Ff_z*D*np.sin(C*np.arctan(B*alpha_f))
    Fr_tire = Fr_z*D*np.sin(C*np.arctan(B*alpha_r))
   
    # Torque Requests
    F_f = F_fl + F_fr
    F_r = F_rl + F_rr
    
    # Longitudinal and Lateral Force and Yaw Moment
    F_x = F_f*np.cos(u[:,1]) - Ff_tire*np.sin(u[:,1]) + F_r - F_roll - F_drag
    F_y = F_f*np.sin(u[:,1]) + Ff_tire*np.cos(u[:,1]) - Fr_tire
    M_z = (F_f*np.sin(u[:,1]) + Ff_tire*np.cos(u[:,1]))*l_f + Fr_tire*l_r + diff_Mz
    
    # State derivative function
    dX = np.array([F_x/m + x[:,1]*x[:,2],\
                  F_y/m - x[:,0]*x[:,2],\
                  M_z/I_z])
    
    return dX

def differential_mpc_model_data_diff_and_load(x,u,p):
    '''x = [vx
        vy
        r]
        
    u = [throttle
            delta]'''
        
    
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
    track_width = p[22]
    diff_gain = p[23]


    # Rolling Resistance
    F_roll = C_roll*m*g
    
    # Drag Force
    F_drag = 1/2*rho*C_d*np.power(x[:,0],2)

    # Longitudinal Load Transfer
    t_bool = (u[:,2] < 0)
    T_front = (~t_bool)*T_max_front + t_bool*(T_brake_front)
    T_rear = (~t_bool)*T_max_rear + t_bool*(T_brake_rear)
    F_f_ax = 2*eta_motor*T_front*GR/r_wheel*u[:,2]
    F_r_ax = 2*eta_motor*T_rear*GR/r_wheel*u[:,2]
    a_x = (F_f_ax*np.cos(u[:,3]) + F_r_ax - F_roll - F_drag)/m + x[:,1]*x[:,2]
    delta_Fz = h_cog/(l_r+l_f)*m*a_x
    
    # Braking or Accel Torque
    t_bool = (u[:,0] < 0)
    T_front = ((~t_bool)*T_max_front + t_bool*(T_brake_front))*u[:,0]
    T_rear = ((~t_bool)*T_max_rear + t_bool*(T_brake_rear))*u[:,0]

    # Get differential torques
    diff_front, diff_rear = differential(u[:,1],x[:,0],diff_gain,a_x)
    T_fl = np.maximum(np.minimum(T_front + diff_front/2,T_max_front),-T_brake_front)
    T_fr = np.maximum(np.minimum(T_front - diff_front/2,T_max_front),-T_brake_front)
    T_rl = np.maximum(np.minimum(T_rear + diff_rear/2,T_max_rear),-T_brake_rear)
    T_rr = np.maximum(np.minimum(T_rear - diff_rear/2,T_max_rear),-T_brake_rear)
    F_fl = eta_motor*T_fl*GR/r_wheel
    F_fr = eta_motor*T_fr*GR/r_wheel
    F_rl = eta_motor*T_rl*GR/r_wheel
    F_rr = eta_motor*T_rr*GR/r_wheel
    diff_Mz = track_width/2*((F_fr-F_fl)+(F_rr-F_rl))

    # Static weight distribution
    Ff_z_static = m * (l_r/(l_f + l_r)) * g
    Fr_z_static = m * (l_f/(l_f + l_r)) * g

    # Downforce Distribution
    F_downforce = 0.5 * rho * C_l * np.power(x[:,0],2)

    Ff_downforce = F_downforce * downforce_front
    Fr_downforce = F_downforce * downforce_rear

    # Slip Angles
    alpha_f = u[:,1] - np.arctan2(x[:,1] + x[:,2]*l_f, x[:,0])
    alpha_r = np.arctan2((x[:,1] - x[:,2]*l_r), x[:,0])

    # Static + Downforce + Longitudinal Load Transfer
    Ff_z = Ff_z_static + Ff_downforce - delta_Fz
    Fr_z = Fr_z_static + Fr_downforce + delta_Fz

    # Lateral Forces
    Ff_tire = Ff_z*D*np.sin(C*np.arctan(B*alpha_f))
    Fr_tire = Fr_z*D*np.sin(C*np.arctan(B*alpha_r))
   
    # Torque Requests
    F_f = F_fl + F_fr
    F_r = F_rl + F_rr
    
    # Longitudinal and Lateral Force and Yaw Moment
    F_x = F_f*np.cos(u[:,1]) - Ff_tire*np.sin(u[:,1]) + F_r - F_roll - F_drag
    F_y = F_f*np.sin(u[:,1]) + Ff_tire*np.cos(u[:,1]) - Fr_tire
    M_z = (F_f*np.sin(u[:,1]) + Ff_tire*np.cos(u[:,1]))*l_f + Fr_tire*l_r + diff_Mz
    
    # State derivative function
    dX = np.array([F_x/m + x[:,1]*x[:,2],\
                  F_y/m - x[:,0]*x[:,2],\
                  M_z/I_z])
    
    return dX

def planar_model_TV(x,u,p,h):
    '''x = [vx
        vy
        r
        vx_prev
        vy_prev]
        
    u = [throttle
            delta]'''
            
    #print(f"vx:{x[0,0]}; vy:{x[0,1]}; r:{x[0,2]}; vx_prev:{x[0,3]}; vy_prev:{x[0,4]}; throttle:{u[0,0]}; delta:{u[0,1]}")
                    
    
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
    track_width = p[22]
    diff_gain = p[23]


    # Rolling Resistance
    F_roll = C_roll*m*g
    
    # Drag Force
    F_drag = 1/2*rho*C_d*np.power(x[:,0],2)
    
    # Braking or Accel Torque
    t_bool = (u[:,0] < 0)
    T_front = ((~t_bool)*T_max_front + t_bool*(T_brake_front))*u[:,0]
    T_rear = ((~t_bool)*T_max_rear + t_bool*(T_brake_rear))*u[:,0]
    
    # Longitudinal and Lateral Load Transfer
    a_x = get_ax(x,h)
    a_y = get_ay(x,h)
    delta_Fz_x = h_cog/(l_r+l_f)*m*a_x/2
    #delta_Fz_y = h_cog/track_width*m*a_y
    delta_Fz_y_f = m*a_y/track_width*(l_r/(l_f+l_r))*0.0368
    delta_Fz_y_r = m*a_y/track_width*(l_f/(l_f+l_r))*0.06
    
    # Differential
    diff_front, diff_rear = differential(u[:,1],x[:,0],diff_gain,a_x)

    # Saturate
    T_fl = np.maximum(np.minimum(T_front + diff_front/2,T_max_front),-T_brake_front)
    T_fr = np.maximum(np.minimum(T_front - diff_front/2,T_max_front),-T_brake_front)
    T_rl = np.maximum(np.minimum(T_rear + diff_rear/2,T_max_rear),-T_brake_rear)
    T_rr = np.maximum(np.minimum(T_rear - diff_rear/2,T_max_rear),-T_brake_rear)
    
    # Get longitudinal Forces
    Fx_fl = eta_motor*T_fl*GR/r_wheel
    Fx_fr = eta_motor*T_fr*GR/r_wheel
    Fx_rl = eta_motor*T_rl*GR/r_wheel
    Fx_rr = eta_motor*T_rr*GR/r_wheel
    
    # # Differential Moment
    # diff_Mz = track_width/2*((Fx_fr-Fx_fl)*np.cos(u[:,1])+(Fx_rr-Fx_rl))
    
    # Static weight distribution
    Ff_z_static = m * (l_r/(l_f + l_r)) * g
    Fr_z_static = m * (l_f/(l_f + l_r)) * g

    # Downforce Distribution
    F_downforce = 0.5 * rho * C_l * np.power(x[:,0],2)

    Ff_downforce = F_downforce * downforce_front
    Fr_downforce = F_downforce * downforce_rear

    # Slip Angles
    # alpha_fl = u[:,1] - np.arctan( (x[:,1] + x[:,2]*l_f) / (x[:,0] - x[:,2]*track_width/2) )
    # alpha_fr = u[:,1] - np.arctan( (x[:,1] + x[:,2]*l_f) / (x[:,0] + x[:,2]*track_width/2) )
    # alpha_rl = -np.arctan( (x[:,1] - x[:,2]*l_r) / (x[:,0] - x[:,2]*track_width/2) )
    # alpha_rr = -np.arctan( (x[:,1] - x[:,2]*l_r) / (x[:,0] + x[:,2]*track_width/2) )
    # alpha_fl = u[:,1] - np.arctan( (x[:,1] + x[:,2]*l_f) / (x[:,0]) )
    # alpha_fr = u[:,1] - np.arctan( (x[:,1] + x[:,2]*l_f) / (x[:,0]) )
    # alpha_rl = -np.arctan( (x[:,1] - x[:,2]*l_r) / (x[:,0])  )
    # alpha_rr = -np.arctan( (x[:,1] - x[:,2]*l_r) / (x[:,0])  )
    # alpha_fl = np.arctan( (x[:,1] + x[:,2]*l_f) / (x[:,0]) ) - u[:,1]
    # alpha_fr = np.arctan( (x[:,1] + x[:,2]*l_f) / (x[:,0]) ) - u[:,1]
    # alpha_rl = np.arctan( (x[:,1] - x[:,2]*l_r) / (x[:,0])  )
    # alpha_rr = np.arctan( (x[:,1] - x[:,2]*l_r) / (x[:,0])  )
    alpha_fl = u[:,1] - np.arctan( (x[:,1] + x[:,2]*l_f) / (x[:,0] - x[:,2]*track_width/2) )
    alpha_fr = u[:,1] - np.arctan( (x[:,1] + x[:,2]*l_f) / (x[:,0] + x[:,2]*track_width/2) )
    alpha_rl = -np.arctan( (x[:,1] - x[:,2]*l_r) / (x[:,0] - x[:,2]*track_width/2)  )
    alpha_rr = -np.arctan( (x[:,1] - x[:,2]*l_r) / (x[:,0] + x[:,2]*track_width/2)  )
    # print(f"alpha_fl:{alpha_fl}; alpha_fr:{alpha_fr}; alpha_rl:{alpha_rl}; alpha_rr:{alpha_rr}")
    
    # Static + Downforce + Longitudinal Load Transfer
    Fz_fl = Ff_z_static/2 + Ff_downforce/2 - delta_Fz_x - delta_Fz_y_f
    Fz_fr = Ff_z_static/2 + Ff_downforce/2 - delta_Fz_x + delta_Fz_y_f
    Fz_rl = Fr_z_static/2 + Fr_downforce/2 + delta_Fz_x - delta_Fz_y_r
    Fz_rr = Fr_z_static/2 + Fr_downforce/2 + delta_Fz_x + delta_Fz_y_r
    # print(f"Ff_z_static:{Ff_z_static}; Fr_z_static:{Fr_z_static}; Ff_downforce:{Ff_downforce}; Fr_downforce:{Fr_downforce}; delta_Fz_x:{delta_Fz_x}; delta_Fz_y_f:{delta_Fz_y_f}; delta_Fz_y_r:{delta_Fz_y_r}")
    # print(f"Fz_fl:{Fz_fl}; Fz_fr:{Fz_fr}; Fz_rl:{Fz_rl}; Fz_rr:{Fz_rr}")

    # Lateral Forces
    Fy_fl = Fz_fl*D*np.sin(C*np.arctan(B*alpha_fl))
    Fy_fr = Fz_fr*D*np.sin(C*np.arctan(B*alpha_fr))
    Fy_rl = Fz_rl*D*np.sin(C*np.arctan(B*alpha_rl))
    Fy_rr = Fz_rr*D*np.sin(C*np.arctan(B*alpha_rr))
    
    # Longitudinal and Lateral Force and Yaw Moment
    F_x = (Fx_fl+Fx_fr)*np.cos(u[:,1]) - (Fy_fl+Fy_fr)*np.sin(u[:,1]) + (Fx_rl+Fx_rr) - F_roll - F_drag
    F_y = (Fx_fl+Fx_fr)*np.sin(u[:,1]) + (Fy_fl+Fy_fr)*np.cos(u[:,1]) + (Fy_rl+Fy_rr)
    # print(f"(Fx_fl+Fx_fr)*np.sin(u[:,1]):{(Fx_fl+Fx_fr)*np.sin(u[:,1])}; (Fy_fl+Fy_fr)*np.cos(u[:,1]):{(Fy_fl+Fy_fr)*np.cos(u[:,1])}; (Fy_rl+Fy_rr):{(Fy_rl+Fy_rr)}")
    # print(f"F_y:{F_y};F_y/m:{F_y/m};-v_x*r:{-x[:,0]*x[:,2]};d_v_y:{F_y/m - x[:,0]*x[:,2]}")
    M_z = (Fx_fl+Fx_fr)*np.sin(u[:,1])*l_f + (Fy_fl+Fy_fr)*np.cos(u[:,1])*l_f - (Fy_rl+Fy_rr)*l_r + track_width/2*((Fx_fr-Fx_fl)*np.cos(u[:,1])+(Fy_fl-Fy_fr)*np.sin(u[:,1])+(Fx_rr-Fx_rl))
    # State derivative function
    dX = np.array([F_x/m + x[:,1]*x[:,2],\
                  F_y/m - x[:,0]*x[:,2],\
                  M_z/I_z])
    # print('\n')
    # print("Fx: %f, Fy: %f, Mz: %f, F_f: %f, F_r : %f, Fy_fl : %f, Fy_fr: %f, Fy_rl: %f, Fy_rr: %f" % (F_x,F_y,M_z,Fx_fl+Fx_fr,Fx_rl+Fx_rr,Fy_fl,Fy_fr,Fy_rl,Fy_rr) )
    
    return dX

def euler_step(x, dX, h):
    return x + h*dX

def differential(delta,v_x,diff_gain,a_x):
    ratio = np.maximum(np.minimum(a_x/9.81,1),-1)*0.16

    diff_Coeffs = [-0.6574, -1.83, 0.0221]

    diff_T = diff_gain*(diff_Coeffs[0]*delta + diff_Coeffs[1]*v_x*delta + diff_Coeffs[2]*np.power(delta,3))

    diff_front = diff_T*(0.4 - ratio)
    diff_rear = diff_T*(0.6 + ratio)

    return diff_front,diff_rear

def get_ax(x,h):
    return (x[:,0]-x[:,3])/h - x[:,1]*x[:,2]

def get_ay(x,h):
    return (x[:,1]-x[:,4])/h + x[:,0]*x[:,2]