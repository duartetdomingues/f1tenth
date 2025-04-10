#include "../include/lmpc_pipeline.hpp"
#define PI 3.14159265359

#include <tf/tf.h>
#include <cmath>
#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <stdio.h>
#include <fstream>

// // for noise testing
// #include <random>

//  z =  [0 - s(t+1) - s(t) 
    //    1 - d(t+1) - d(t)
    //    2 - delta(t+1) - delta(t)
    //    3 - s(t) 
    //    4 - d(t)
    //    5 - delta(t)
    //    6 - X 
    //    7 - Y 
    //    8 - Psi 
    //    9 - vx 
    //    10 - vy 
    //    11 - r]

// p =  [0 - alpha_CL
    //   1 - alpha_L
    //   2 - q_v_y 
    //   3 - e_CL_exp
    //   4 - d_max
    //   5 - beta_steer
    //   6 - lambda
    //   7 - v_max 
    //   8 - q_v_max
    //   9 - l_f
    //   10 - l_r    
    //   11 - m
    //   12 - I_z
    //   13 - T_max_front
    //   14 - T_max_rear
    //   15 - T_brake_front
    //   16 - T_brake_rear 
    //   17 - GR
    //   18 - eta_motor
    //   19 - r_wheel
    //   20 - g
    //   21 - C_roll
    //   22 - rho
    //   23 - lambda_blend_min
    //   24 - lambda_blend_max
    //   25 - C_d
    //   26 - C_l
    //   27 - B
    //   28 - C
    //   29 - D
    //   30 - downforce_front
    //   31 - downforce_rear]

/****************************************
*             General Tasks             *
****************************************/

// Constructor
LearningMpcc::LearningMpcc()
{
    ROS_INFO("Constructing learning_mpcc");

    if (!ros::param::get("/load_map", _loadMap)) {
		ROS_WARN_STREAM("Could not load /load_map. Default value is false");
		_loadMap = true;
	}

    if (!ros::param::get("learning_mpcc/mission_config", _mission_config))
    {
        ROS_WARN("Learning MPCC: Path to lmpc not provided. Using default path");
        _mission_config = "trackdrive";
    }

    double num_of_threads;
    if (!ros::param::get("learning_mpcc/num_of_threads",  num_of_threads))
    {
        ROS_WARN("Learning MPCC: number of threads dedicated to solver not provided. Using 4 threads by default.");
        _params.num_of_threads = 4;
    }
    else
    {
        _params.num_of_threads = (int)num_of_threads;
    }
    // ROS_WARN("Solver num of threads: %d",_params.num_of_threads);

    if (!ros::param::get("learning_mpcc/solver_timeout", _params.solver_timeout))
    {
        ROS_WARN("Learning MPCC: solver timeout not provided. Using 45 ms timeout by default.");
        _params.solver_timeout = 0.045f;
    }
    // ROS_WARN("Solver timeout: %f",_params.solver_timeout);

    if (!ros::param::get("learning_mpcc/controller_freq", _controller_freq))
    {
        ROS_WARN("Learning MPCC: controller freq not provided. Using 20 Hz by default.");
        _controller_freq = 20.0f;
    }
    if (!ros::param::get("learning_mpcc/track_length", _track_length))
    {
        ROS_WARN("Learning MPCC: track length not defined. Using 309.40 Hz by default.");
        _track_length = 309.40;
    }

    // controller mpc_params
    if (!ros::param::get("learning_mpcc/mpc_params/objective/alpha_CL", _mpcParams.alpha_CL))
    {
        ROS_WARN("Learning MPCC: alpha_CL not provided. Using alpha_CL = 200.0 by default.");
        _mpcParams.alpha_CL = 200.0f;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/alpha_L", _mpcParams.alpha_L))
    {
        ROS_WARN("Learning MPCC: alpha_L not provided. Using alpha_L = 1000.0 by default.");
        _mpcParams.alpha_L = 1000.0f;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/q_r", _mpcParams.q_r))
    {
        ROS_WARN("Learning MPCC: q_v_y not provided. Using q_v_y = 100.0 by default.");
        _mpcParams.q_r = 100.0f;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/e_CL_exp", _mpcParams.e_CL_exp))
    {
        ROS_WARN("Learning MPCC: e_CL_exp not provided. Using e_CL_exp = 2 by default.");
        _mpcParams.e_CL_exp = 2;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/d_max", _mpcParams.d_max))
    {
        ROS_WARN("Learning MPCC: d_max not provided. Using d_max = 0.3 by default.");
        _mpcParams.d_max = 1.0f;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/beta_steer", _mpcParams.beta_steer))
    {
        ROS_WARN("Learning MPCC: beta_steer not provided. Using beta_steer = 400.0 by default.");
        _mpcParams.beta_steer = 1750.0f;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/lambda", _mpcParams.lambda))
    {
        ROS_WARN("Learning MPCC: lambda not provided. Using lambda = 200.0 by default.");
        _mpcParams.lambda = 70.0f;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/v_max", _mpcParams.v_max))
    {
        ROS_WARN("Learning MPCC: v_max not provided. Using v_max = 1.0 m/s by default.");
        _mpcParams.v_max = 10.0f;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/v_max_trackdrive", _mpcParams.v_max_trackdrive))
    {
        ROS_WARN("Learning MPCC: v_max_trackdrive not provided. Using v_max = 25.0 m/s by default.");
        _mpcParams.v_max_trackdrive = 25.0f;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/q_v_max", _mpcParams.q_v_max))
    {
        ROS_WARN("Learning MPCC: q_v_max not provided. Using q_v_max = 6.0 by default.");
        _mpcParams.q_v_max = 10.0f;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/z", _mpcParams.z))
    {
        ROS_WARN("Learning MPCC: z not provided. Using z = 12 by default.");
        _mpcParams.z = 12;
    }
    if (!ros::param::get("learning_mpcc/mpc_params/objective/p", _mpcParams.p))
    {
        ROS_WARN("Learning MPCC: z not provided. Using z = 12 by default.");
        _mpcParams.p = 32;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/N", _mpcParams.N))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.N = 41;
    }

    if (!ros::param::get("learning_mpcc/model_params/lambda_blend_min", _mpcParams.lambda_blend_min))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.lambda_blend_min = 3.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/lambda_blend_max", _mpcParams.lambda_blend_max))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.lambda_blend_max = 4.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/l_r", _mpcParams.l_r))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.l_r = 0.88;
    }

    if (!ros::param::get("learning_mpcc/model_params/l_f", _mpcParams.l_f))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.l_f = 0.66;
    }

    if (!ros::param::get("learning_mpcc/tyre_params/D", _mpcParams.D))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.D = 1.0279;
    }

    if (!ros::param::get("learning_mpcc/tyre_params/D_trackdrive", _mpcParams.D_trackdrive))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.D_trackdrive = 1.0279;
    }

    if (!ros::param::get("learning_mpcc/tyre_params/C", _mpcParams.C))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.C = 2.0;
    }

    if (!ros::param::get("learning_mpcc/tyre_params/B", _mpcParams.B))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.B = -10.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/downforce_front", _mpcParams.downforce_front))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.downforce_front = 0.42;
    }

    if (!ros::param::get("learning_mpcc/model_params/downforce_rear", _mpcParams.downforce_rear))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.downforce_rear = 0.58;
    }

    if (!ros::param::get("learning_mpcc/model_params/g", _mpcParams.g))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.g = 9.81;
    }

    if (!ros::param::get("learning_mpcc/model_params/rho", _mpcParams.rho))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.rho = 1.225;
    }

    if (!ros::param::get("learning_mpcc/model_params/C_l", _mpcParams.C_l))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.C_l = 4.64;
    }

    if (!ros::param::get("learning_mpcc/model_params/C_d", _mpcParams.C_d))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.C_d = 1.78;
    }

    if (!ros::param::get("learning_mpcc/model_params/m", _mpcParams.m))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.m = 238.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/I_z", _mpcParams.I_z))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.I_z = 180.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/T_max_front", _mpcParams.T_max_front))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.T_max_front = 4.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/T_max_rear", _mpcParams.T_max_rear))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.T_max_rear = 6.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/T_brake_front", _mpcParams.T_brake_front))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.T_brake_front = 9.8;
    }
    
    if (!ros::param::get("learning_mpcc/model_params/T_brake_rear", _mpcParams.T_brake_rear))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.T_brake_rear = 4.2;
    }

    if (!ros::param::get("learning_mpcc/model_params/GR", _mpcParams.GR))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.GR = 15.21;
    }

    if (!ros::param::get("learning_mpcc/model_params/eta_motor", _mpcParams.eta_motor))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.eta_motor = 0.85;
    }

    if (!ros::param::get("learning_mpcc/model_params/C_roll", _mpcParams.C_roll))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.C_roll = 0.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/r_wheel", _mpcParams.r_wheel))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.r_wheel = 0.235;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/beta_throttle", _mpcParams.beta_throttle))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.beta_throttle = 0.0;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/beta_throttle_trackdrive", _mpcParams.beta_throttle_trackdrive))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.beta_throttle_trackdrive = 0.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/h_cog", _mpcParams.h_cog))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.h_cog = 0.0;
    }

    if (!ros::param::get("learning_mpcc/tyre_params/mu_x", _mpcParams.mu_x))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.mu_x = 0.0;
    }

    if (!ros::param::get("learning_mpcc/tyre_params/mu_y", _mpcParams.mu_y))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.mu_y = 0.0;
    }

    if (!ros::param::get("learning_mpcc/tyre_params/mu_x_trackdrive", _mpcParams.mu_x_trackdrive))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.mu_x_trackdrive = 0.0;
    }

    if (!ros::param::get("learning_mpcc/tyre_params/mu_y_trackdrive", _mpcParams.mu_y_trackdrive))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.mu_y_trackdrive = 0.0;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/ineq_limits/e_CL_max", _mpcParams.e_CL_max))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.e_CL_max = 0.0;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/ineq_limits/delta_s_max", _mpcParams.delta_s_max))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.delta_s_max = 0.0;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/ineq_limits/throttle_max", _mpcParams.throttle_max))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.throttle_max = 0.0;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/ineq_limits/steering_max", _mpcParams.steering_max))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.steering_max = 0.0;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/ineq_limits/delta_throttle_max", _mpcParams.delta_throttle_max))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.delta_throttle_max = 0.0;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/ineq_limits/delta_steering_max", _mpcParams.delta_steering_max))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.delta_steering_max = 0.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/length", _mpcParams.length))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.length = 0.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/width", _mpcParams.width))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.width = 0.0;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/ineq_limits/safety_margin", _mpcParams.safety_margin))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.safety_margin = 0.0;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/ineq_limits/dash", _mpcParams.dash))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.dash = true;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/ineq_limits/dash_factor", _mpcParams.dash_factor))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.dash_factor = 5.2;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/beta_epsilon", _mpcParams.beta_epsilon))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.beta_epsilon = 0.0;
    }

    if (!ros::param::get("learning_mpcc/NN_config/NN", _mpcParams.NN_flag))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.NN_flag = 0.0;
    }

    if (!ros::param::get("learning_mpcc/NN_config/NN_params", _mpcParams.NN_params))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.NN_params = 1991;
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/e_CL_after_train", _mpcParams.e_CL_after_train))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.e_CL_after_train = 12.5;
    }

    if (!ros::param::get("learning_mpcc/NN_config/model_id", _model_id))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _model_id = 20.0;
    }

    if (!ros::param::get("learning_mpcc/NN_config/model_name", _model_name))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _model_name = "model_20";
    }

    if (!ros::param::get("learning_mpcc/mpc_params/objective/beta_psi", _mpcParams.beta_psi))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.beta_psi = 0.1;
    }

    if (!ros::param::get("learning_mpcc/model_params/track_width", _mpcParams.track_width))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.track_width = 1.0;
    }

    if (!ros::param::get("learning_mpcc/model_params/diff_gain", _mpcParams.diff_gain))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.diff_gain = 1.0;
    }
    _diffGain.data = _mpcParams.diff_gain;

    if (!ros::param::get("learning_mpcc/mpc_params/objective/spline", _mpcParams.spline_points))
    {
        ROS_WARN("Learning MPCC: N not provided. Using N = 41.0 by default.");
        _mpcParams.spline_points = 15;
    }

    if (!ros::param::get("learning_mpcc/lmpc_path", _lmpcPath))
    {
        ROS_WARN("Learning MPCC: Path to lmpc not provided. Using default path");
        _lmpcPath = "/home/fst/autonomous-systems/src/learning_mpcc";
    }

    if (!ros::param::get("learning_mpcc/simul", _simul))
    {
        ROS_WARN("Learning MPCC: Path to lmpc not provided. Using default path");
        _simul = 0;
    }

    // data writing options
    if (!ros::param::get("learning_mpcc/data_file/write_data_file", _write_data_file))
    {
        ROS_WARN("Learning MPCC: Write to data file not provided. Not writing by default");
        _write_data_file = false;
    }
    if (_write_data_file)
    {
        std::string data_file_name;
        if (!ros::param::get("learning_mpcc/data_file/name", data_file_name))
        {
            ROS_WARN("Learning MPCC: data file name not provided. Writing to \"lmpcc_data\" by default");
            data_file_name = "lmpcc_data.txt";
        }
        _data_file.open(data_file_name, std::ios_base::trunc);
        if (!_data_file.is_open())
        {
            ROS_ERROR("Learning MPCC: UNABLE TO OPEN DATA WRITING FILE.");
            _write_data_file = false;
        }
        if (_write_data_file)
            ROS_INFO_STREAM("Learning MPCC: the data file named '" << data_file_name << "' will be in the .ros folder");
    }
    ROS_WARN("NOT WRITING TO FILE");

    _dash.steering_angle = 0.0f;
    _controlCmd.throttle = 0.1f;
    _controlCmd.steering_angle = 0.0f;

    _prevControlCmd.throttle = 0.0f;
    _prevControlCmd.steering_angle = 0.0f;
    _prevDash.steering_angle = 0.0f;

    _currentControlCmd.steering_angle = 0.0f;
    _currentControlCmd.throttle = 0.0f;
    _currentDash.steering_angle = 0.0f;
    ROS_WARN("Setting Throttle and Steering to 0");

    // Warm start up initialization
    for (int i = 0; i < _mpcParams.z*(_mpcParams.N-2) + _mpcParams.z-2 + _mpcParams.z-6; i++)
    {
        _params.x0[i] = 0.01f;
    }
    ROS_WARN("Init x0");

    // Parameter initialization
    for (int i = 0; i < _mpcParams.N * (_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2); i += (_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2))
    {
        _params.all_parameters[i] = _mpcParams.alpha_CL;
        _params.all_parameters[i+1] = _mpcParams.alpha_L;
        _params.all_parameters[i+2] = _mpcParams.q_r;
        _params.all_parameters[i+3] = _mpcParams.e_CL_exp;
        _params.all_parameters[i+4] = _mpcParams.d_max;
        _params.all_parameters[i+5] = _mpcParams.beta_steer;
        _params.all_parameters[i+6] = _mpcParams.lambda;
        _params.all_parameters[i+7] = _mpcParams.v_max;
        _params.all_parameters[i+8] = _mpcParams.q_v_max;
        _params.all_parameters[i+9] = _mpcParams.l_f;
        _params.all_parameters[i+10] = _mpcParams.l_r;
        _params.all_parameters[i+11] = _mpcParams.m;
        _params.all_parameters[i+12] = _mpcParams.I_z;
        _params.all_parameters[i+13] = _mpcParams.T_max_front;
        _params.all_parameters[i+14] = _mpcParams.T_max_rear;
        _params.all_parameters[i+15] = _mpcParams.T_brake_front;
        _params.all_parameters[i+16] = _mpcParams.T_brake_rear;
        _params.all_parameters[i+17] = _mpcParams.GR;
        _params.all_parameters[i+18] = _mpcParams.eta_motor;
        _params.all_parameters[i+19] = _mpcParams.r_wheel;
        _params.all_parameters[i+20] = _mpcParams.g;
        _params.all_parameters[i+21] = _mpcParams.C_roll;
        _params.all_parameters[i+22] = _mpcParams.rho;
        _params.all_parameters[i+23] = _mpcParams.lambda_blend_min;
        _params.all_parameters[i+24] = _mpcParams.lambda_blend_max;
        _params.all_parameters[i+25] = _mpcParams.C_d;
        _params.all_parameters[i+26] = _mpcParams.C_l;
        _params.all_parameters[i+27] = _mpcParams.B;
        _params.all_parameters[i+28] = _mpcParams.C;
        _params.all_parameters[i+29] = _mpcParams.D;
        _params.all_parameters[i+30] = _mpcParams.downforce_front;
        _params.all_parameters[i+31] = _mpcParams.downforce_rear;
        _params.all_parameters[i+32] = _mpcParams.beta_throttle;
        _params.all_parameters[i+33] = _mpcParams.h_cog;
        _params.all_parameters[i+34] = _mpcParams.mu_x;
        _params.all_parameters[i+35] = _mpcParams.mu_y;
        _params.all_parameters[i+36] = _mpcParams.e_CL_max;
        _params.all_parameters[i+37] = _mpcParams.delta_s_max;
        _params.all_parameters[i+38] = _mpcParams.throttle_max;
        _params.all_parameters[i+39] = _mpcParams.steering_max;
        _params.all_parameters[i+40] = _mpcParams.delta_throttle_max; 
        _params.all_parameters[i+41] = _mpcParams.delta_steering_max;
        _params.all_parameters[i+42] = _mpcParams.width;
        _params.all_parameters[i+43] = _mpcParams.length;
        _params.all_parameters[i+44] = _mpcParams.safety_margin;
        _params.all_parameters[i+45] = _mpcParams.beta_epsilon;
        _params.all_parameters[i+46] = _mpcParams.N;
        _params.all_parameters[i+47] = _controller_freq;
        _params.all_parameters[i+48] = _mpcParams.NN_flag;
        _params.all_parameters[i+49] = _mpcParams.p;
        _params.all_parameters[i+50] = _mpcParams.beta_psi;
        _params.all_parameters[i+51] = _mpcParams.track_width;
        _params.all_parameters[i+52] = _mpcParams.diff_gain;
    }

    for (int i = 0; i < _mpcParams.p-1;i++)
    {
        ROS_WARN("Parameter %d: %f", i, _params.all_parameters[i]);
    }

    // Load NN parameters
    // Load the model
    _pathModel = _lmpcPath + "/AI/saved_models/" + _model_name + ".pth";
    ROS_WARN("%s",_pathModel);

    torch::jit::script::Module model;
    try {
        model = torch::jit::load(_pathModel);
        ROS_WARN("Model loaded: %s",_pathModel.c_str());
    } catch (const c10::Error& e) {
        std::cerr << "Error loading the model: " << e.msg() << std::endl;
        ROS_ERROR("Error loading the model: %s", e.msg());
    }

    // Access the parameters of the model
    auto params = model.parameters();
    // ROS_WARN("Parameters: %d", params.size());
    std::vector<at::Tensor> parameters;
    for (const auto& params : model.parameters()) {
        parameters.push_back(params);
    }
    
    // Load the weights and biases
    auto tmp = parameters[0].transpose(0,1).contiguous().view({-1});
    std::vector<float> fc1_kernel (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    tmp = parameters[1].view({-1}).contiguous();
    std::vector<float> fc1_biases (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    tmp = parameters[2].transpose(0,1).contiguous().view({-1});
    // std::vector<float> fc2_kernel (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    // tmp = parameters[3].view({-1}).contiguous();
    // std::vector<float> fc2_biases (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    // tmp = parameters[4].transpose(0,1).contiguous().view({-1});
    std::vector<float> rnn_kernel_ih (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    tmp = parameters[3].transpose(0,1).contiguous().view({-1});
    std::vector<float> rnn_kernel_hh (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    tmp = parameters[4].view({-1}).contiguous();
    std::vector<float> rnn_biases_ih (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    tmp = parameters[5].view({-1}).contiguous();
    std::vector<float> rnn_biases_hh (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    tmp = parameters[6].transpose(0,1).contiguous().view({-1});
    // std::vector<float> fc2_reversed_kernel (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    // tmp = parameters[9].view({-1}).contiguous();
    // std::vector<float> fc2_reversed_biases (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    // tmp = parameters[10].transpose(0,1).contiguous().view({-1});
    std::vector<float> fc1_reversed_kernel (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    tmp = parameters[7].view({-1}).contiguous();
    std::vector<float> fc1_reversed_biases (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    tmp = parameters[8].transpose(0,1).contiguous().view({-1});
    std::vector<float> output_kernel (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());
    tmp = parameters[9].view({-1}).contiguous();
    std::vector<float> output_biases (tmp.data_ptr<float>(), tmp.data_ptr<float>() + tmp.numel());

    // Replace the weights in the network
    
    for (size_t it = 0; it < _mpcParams.N; it++) {
        size_t ind = 0;
        for (size_t i = 0; i < fc1_kernel.size(); i++) {
            if (i == 0 && it == 0)
                ROS_WARN("fc1_kernel[0] = %f", fc1_kernel[i]);
            _params.all_parameters[i+_mpcParams.p+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = fc1_kernel[i];
        }
        ind = ind + fc1_kernel.size();
        for (size_t i = 0; i < fc1_biases.size(); i++) {
            if (i == 0 && it == 0)
                ROS_WARN("fc1_biases[0] = %f", fc1_biases[i]);
            _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = fc1_biases[i];
        }
        ind = ind + fc1_biases.size();
        // for (size_t i = 0; i < fc2_kernel.size(); i++) {
        //     if (i == 0 && it == 0)
        //         ROS_WARN("fc2_kernel[0] = %f", fc2_kernel[i]);
        //     _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = fc2_kernel[i];
        // }
        // ind = ind + fc2_kernel.size();
        // for (size_t i = 0; i < fc2_biases.size(); i++) {
        //     if (i == 0 && it == 0)
        //         ROS_WARN("fc2_biases[0] = %f", fc2_biases[i]);
        //     _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = fc2_biases[i];
        // }
        // ind = ind + fc2_biases.size();
        for (size_t i = 0; i < rnn_kernel_ih.size(); i++) {
            if (i == 0 && it == 0)
                ROS_WARN("rnn_kernel[0] = %f", rnn_kernel_ih[i]);
            _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = rnn_kernel_ih[i];
        }
        ind = ind + rnn_kernel_ih.size();
        for (size_t i = 0; i < rnn_kernel_hh.size(); i++) {
            if (i == 0 && it == 0)
                ROS_WARN("rnn_recurrent_kernel[0] = %f", rnn_kernel_hh[i]);
            _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = rnn_kernel_hh[i];
        }
        ind = ind + rnn_kernel_hh.size();
        for (size_t i = 0; i < rnn_biases_ih.size(); i++) {
            if (i == 0 && it == 0)
                ROS_WARN("rnn_biases_ih[0] = %f", rnn_biases_ih[i]);
            _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = rnn_biases_ih[i];
        }
        ind = ind + rnn_biases_ih.size();
        for (size_t i = 0; i < rnn_biases_hh.size(); i++) {
            if (i == 0 && it == 0)
                ROS_WARN("rnn_biases_hh[0] = %f", rnn_biases_hh[i]);
            _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = rnn_biases_hh[i];
        }
        ind = ind + rnn_biases_hh.size();
        // for (size_t i = 0; i < fc2_reversed_kernel.size(); i++) {
        //     if (i == 0 && it == 0)
        //         ROS_WARN("fc2_reversed_kernel[0] = %f", fc2_reversed_kernel[i]);
        //     _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = fc2_reversed_kernel[i];
        // }
        // ind = ind + fc2_reversed_kernel.size();
        // for (size_t i = 0; i < fc2_reversed_biases.size(); i++) {
        //     if (i == 0 && it == 0)
        //         ROS_WARN("fc2_reversed_biases[0] = %f", fc2_reversed_biases[i]);
        //     _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = fc2_reversed_biases[i];
        // }
        // ind = ind + fc2_reversed_biases.size();
        for (size_t i = 0; i < fc1_reversed_kernel.size(); i++) {
            if (i == 0 && it == 0)
                ROS_WARN("fc1_reversed_kernel[0] = %f", fc1_reversed_kernel[i]);
            _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = fc1_reversed_kernel[i];
        }
        ind = ind + fc1_reversed_kernel.size();
        for (size_t i = 0; i < fc1_reversed_biases.size(); i++) {
            if (i == 0 && it == 0)
                ROS_WARN("fc1_reversed_biases[0] = %f", fc1_reversed_biases[i]);
            _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = fc1_reversed_biases[i];
        }
        ind = ind + fc1_reversed_biases.size();
        for (size_t i = 0; i < output_kernel.size(); i++) {
            if (i == 0 && it == 0)
                ROS_WARN("output_kernel[0] = %f", output_kernel[i]);
            _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = output_kernel[i];
        }
        ind = ind + output_kernel.size();
        for (size_t i = 0; i < output_biases.size(); i++) {
            if (i == 0 && it == 0)
                ROS_WARN("output_biases[0] = %f", output_biases[i]);
            _params.all_parameters[i+_mpcParams.p+ind+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = output_biases[i];
        }
    }

    // for (int i = _mpcParams.p; i < _mpcParams.p+_mpcParams.NN_params;i++)
    // {
    //     ROS_WARN("NN Parameter %d: %f", i, _params.all_parameters[i]);
    // }


    // Marker initialization
    _newCPMsg.data.clear(); // cone hit message initialization

    _marker_prediction.scale.x = 0.20f;
    _marker_prediction.scale.y = 0.20f;
    _marker_prediction.scale.z = 0.20f;
    _marker_prediction.color.a = 0.75f;
    _marker_prediction.color.r = 0;
    _marker_prediction.color.g = 1;
    _marker_prediction.color.b = 0;
    _marker_prediction.pose.orientation.w = 1.0f;
    _marker_prediction.type = visualization_msgs::Marker::LINE_STRIP;
    _marker_prediction.lifetime = ros::Duration(1/_controller_freq);
    _marker_prediction.action = visualization_msgs::Marker::ADD;
    _marker_prediction.pose.position.z = 0.25f;

    _marker_prediction_text.scale.z = 0.20f;
    _marker_prediction_text.color.a = 0.75f;
    _marker_prediction_text.color.r = 0;
    _marker_prediction_text.color.g = 0;
    _marker_prediction_text.color.b = 0;
    _marker_prediction_text.pose.orientation.w = 1.0f;
    _marker_prediction_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;;
    _marker_prediction_text.lifetime = ros::Duration(_controller_freq);
    _marker_prediction_text.action = visualization_msgs::Marker::ADD;

    _marker_velocity.scale.x = 0.20f;
    _marker_velocity.scale.y = 0.20f;
    _marker_velocity.color.a = 0.75f;
    _marker_velocity.color.r = 1.0f;
    _marker_velocity.color.g = 0.0f;
    _marker_velocity.color.b = 0.0f;
    _marker_velocity.type = visualization_msgs::Marker::CYLINDER;
    _marker_velocity.lifetime = ros::Duration(1/_controller_freq);
    // _marker_velocity.lifetime = ros::Duration(_params.solver_timeout);
    _marker_velocity.action = visualization_msgs::Marker::ADD;

    _marker_spline.scale.x = 0.20f;
    _marker_spline.scale.y = 0.20f;
    _marker_spline.scale.z = 0.20f;
    _marker_spline.color.a = 0.75f;
    _marker_spline.color.r = 0;
    _marker_spline.color.g = 0;
    _marker_spline.color.b = 1;
    _marker_spline.type = visualization_msgs::Marker::SPHERE;
    _marker_spline.lifetime = ros::Duration(1/_controller_freq);
    _marker_spline.action = visualization_msgs::Marker::ADD;

    for (int i = 0; i < _safe_ctrl_actions.rows(); i++)
    {
        for (int j = 0; j < _safe_ctrl_actions.cols(); j++)
        {
            if (i == 0){
                _safe_ctrl_actions(i, j) = 0.1;
            }
            else{
                _safe_ctrl_actions(i, j) = 0.0;
            }
            
        }
    }

    ROS_WARN("LMPC Constructor finished.");
}


void LearningMpcc::loadTrackdriveParams() {

    for (int i = 0; i < _mpcParams.N * (_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2); i += (_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2))
    {
        _params.all_parameters[i+7] = _mpcParams.v_max_trackdrive;
        _params.all_parameters[i+29] = _mpcParams.D_trackdrive;
        _params.all_parameters[i+32] = _mpcParams.beta_throttle_trackdrive;
        _params.all_parameters[i+34] = _mpcParams.mu_x_trackdrive;
        _params.all_parameters[i+35] = _mpcParams.mu_y_trackdrive;
    }
    
}

/******************************** Getters ********************************/
common_msgs::ControlCmd const &LearningMpcc::getControlCommand() const { return _controlCmd; }
visualization_msgs::MarkerArray const &LearningMpcc::getPredictions() const { return _predictions; }
visualization_msgs::MarkerArray const &LearningMpcc::getPredictionsText() const { return _predictions_text; }
visualization_msgs::MarkerArray const &LearningMpcc::getPredictionsVelocities() const { return _predictions_velocity; }
visualization_msgs::MarkerArray const &LearningMpcc::getPosition() const { return _lastlap; }
std_msgs::Float32MultiArray const &LearningMpcc::getLearningData() const { return _learningDataMsg; }
std_msgs::UInt8MultiArray const &LearningMpcc::getNewCP() const { return _newCPMsg; }
std_msgs::Float32 const &LearningMpcc::getThrottle() const {return _throttleViz; }
std_msgs::Float32 const &LearningMpcc::getSteeringAngle() const {return _steeringAngleViz; }
common_msgs::MPCInfo const &LearningMpcc::getMPCInfo() const { return _mpcInfo; }
common_msgs::ModelInfoCompare const &LearningMpcc::getMPCModel() const { return _modelInfo; }
common_msgs::Output const &LearningMpcc::getOutputMPC() const { return _outputDebug; }
std_msgs::Float64 LearningMpcc::getCPUtime() { return _CPUtime; }
common_msgs::TireModel const &LearningMpcc::getTire() const { return _tireModel; }
common_msgs::NormalForces const &LearningMpcc::getNormalForces() const { return _normalForces; }
common_msgs::FrictionEllipse const &LearningMpcc::getFrictionEllipseFront() const { return _frictionEllipseFront; }
common_msgs::FrictionEllipse const &LearningMpcc::getFrictionEllipseRear() const { return _frictionEllipseRear; }
common_msgs::NNData const &LearningMpcc::getNNData() const { return _nnData; }
std_msgs::Float64 &LearningMpcc::getDiffGain() { return _diffGain; }
visualization_msgs::Marker const &LearningMpcc::getMarkerPointToFollow() const { return _purePursuit.getMarkerPointToFollow(); }
visualization_msgs::Marker const &LearningMpcc::getMarkerPointAhead() const { return _purePursuit.getMarkerPointAhead(); }
visualization_msgs::MarkerArray const &LearningMpcc::getSplineMarkersMatlab() const { return _splineMarkersMatlab; }

/******************************** SETTERS ********************************/


void LearningMpcc::setInitialGuess()
{
    // TESTAR ESTA IMPLEMENTAÇÃO COM A IMPLEMENTAÃO PASSADA E VER DIFERENÇAS

    if(_exit_code == 1 || _exit_code == 0 || _exit_code == 2){
        // ROS_WARN("Setting informed initial guess");
        for (int i=0;i<_mpcParams.z * (_mpcParams.N-2);i++){
            _params.x0[i] = _output.z[i];
        }
        for (int i=0; i< _mpcParams.z-2; i++){ //THIS CAN BE CHANGED IN THE FUTURE TO ACTUALLY UPDATE X0 WITH TH NEXT TIMSTEP INSTEAD OF THE CURRENT, THIS MAY BETTER APPROXIMATTE THE REAL SOLUTION AND THEREFORE CNVERGE FASTER
            _params.x0[i + _mpcParams.z * (_mpcParams.N-2)] = _output.zN_1[i];
        }
        for (int i=0; i< _mpcParams.z-6; i++){ 
            _params.x0[i + (_mpcParams.N-2) * _mpcParams.z + _mpcParams.z-2] = _output.zN[i];
        }
    }else{
        // ROS_WARN("Setting default initial guess");
        for (int i = 0; i < _mpcParams.z + (_mpcParams.N-2) * _mpcParams.z-2 + _mpcParams.z-6; i++)
        {
            _params.x0[i] = 0.01f;
        }
    }


    if (_fail_counter == 0)
    {
        for (int i = 0; i < _safe_ctrl_actions.rows(); i++)
        {
            for (int j = 0; j < _safe_ctrl_actions.cols(); j++)
            {
                _safe_ctrl_actions(i, j) = _output.z[i + _mpcParams.z * (j+2) + 3]; // Skip first control input since already applied 
            }
        }
    }
}

/**
*	Name: setCurrentVelocity.
*	Description: Executed by a callback to update the car's velocity.
*	Inputs: Car velocity message of the type common_msgs::CarVelocity
*	Output: void
*/
void LearningMpcc::setCurrentVelocity(const common_msgs::CarVelocity &velMsg)
{
    _localStatePrev = _localState;

    _localState.vx = velMsg.velocity.x;
    _localState.vy = velMsg.velocity.y;
    _localState.r = velMsg.velocity.theta;
    

    _purePursuit.setVelocity(velMsg);

}

void LearningMpcc::setCurrentAcceleration(const sensor_msgs::Imu &accelMsg)
{
    _acceleration.ax = accelMsg.linear_acceleration.x;
    _acceleration.ay = accelMsg.linear_acceleration.y;
    // ROS_WARN("Setting acceleration; x: %f, y: %f", _acceleration.ax, _acceleration.ay);
}

void LearningMpcc::setLoopClosure(const std_msgs::Bool &loopClosure) {

	if (!_loopClosure && loopClosure.data && _mission_config == "trackdrive") { //  && !_loadMap ... || (_counterLoop != 0 && _counterLoop < 10)
		_loopClosure = loopClosure.data; 
        _counterLoop += 1;
		loadTrackdriveParams();
	}
	else {
		_loopClosure = loopClosure.data;
	}
}

void LearningMpcc::setDashSteering(const common_msgs::ControlCmd &dash)
{
    _dash.steering_angle = dash.steering_angle;
}

/**
*	Name: setSlamPose.
*	Description: Executed by a callback to update the car's pose.
*	Inputs: Odometry message of the type nav_msgs::Odometry
*	Output: void
*/
void LearningMpcc::setSlamPose(const nav_msgs::Odometry &slamPoseMsg)
{
    // _globalStateSlam.psi = getYawFromQuaternion(slamPoseMsg);

    _globalState.x = slamPoseMsg.pose.pose.position.x;
    _globalState.y = slamPoseMsg.pose.pose.position.y;
    _globalState.header = slamPoseMsg.header;

    double delta = ros::Time::now().toSec() - slamPoseMsg.header.stamp.toSec();
    double dx = _localState.vx*delta;
    double dy = _localState.vy*delta;

    // ROS_WARN("Delta Time: %f ms", delta*1000);

    float psi_clamped = getYawFromQuaternion(slamPoseMsg);
    float diff = psi_clamped - _last_psi_clamped;
    if (diff > 3)
    {
        // anti clockwise wind
        _winding_counter--;
        //ROS_INFO("winding counter = %d", _winding_counter);
    }
    else if (diff < -3)
    {
        // clockwise wind
        _winding_counter++;
        //ROS_INFO("winding counter = %d", _winding_counter);
    }

    _globalState.psi = _winding_counter * TAU + psi_clamped;

    _last_psi_clamped = psi_clamped;

    _purePursuit.setOdometry(slamPoseMsg);

}

void LearningMpcc::changeLmpcState(const common_msgs::ControlMonitoring &msg) {

	// ordered by priority
	if (msg.require_braking) {
		_controlState = ControlState::BRAKING_MODE;
	}
	else if (!msg.control_on) {
		_controlState = ControlState::CONTROL_OFF;
	}
	else {
		_controlState = ControlState::CONTROL_ON;
	}
}


/********  End of General Tasks  ********/

void LearningMpcc::run() {
    // _prevControlCmd = _currentControlCmd;
    // _currentControlCmd = _controlCmd;

    if (_controlState == ControlState::CONTROL_ON){
        runAlgorithm();
        _purePursuit.runAlgorithm();
        // _controlCmd = _purePursuit.getControlCmd();
        _publish = true;
    } else if (_controlState == ControlState::BRAKING_MODE){
        //_purePursuit.runAlgorithm();
        //_controlCmd = _purePursuit.getControlCmd();
        runAlgorithm();
        _controlCmd.throttle = -0.8;
        _publish = true;
    } else if (_controlState == ControlState::CONTROL_OFF) {
        runAlgorithm();
        _publish = false;
        _controlCmd.throttle = 0;
        _controlCmd.steering_angle = 0;
    } else if (_controlState == ControlState::INSPECTION) {
        runAlgorithm();
        _publish = true;
		inspectionControlCmd();
    }

    // getNearestIndex();
    // _nnData.header.stamp = _mpcInfo.header.stamp;
    // _nnData.control_cmd = _currentControlCmd;
    // _nnData.prev_control_cmd = _prevControlCmd;
    // _nnData.car_pose.x = _globalState.x;
    // _nnData.car_pose.y = _globalState.y;
    // _nnData.car_pose.theta = _globalState.psi;
    // _nnData.car_velocity.velocity.x = _localState.vx;
    // _nnData.car_velocity.velocity.y = _localState.vy;
    // _nnData.car_velocity.velocity.theta = _localState.r;
    // _nnData.prev_car_velocity.velocity.x = _localStatePrev.vx;
    // _nnData.prev_car_velocity.velocity.y = _localStatePrev.vy;
    // _nnData.prev_car_velocity.velocity.theta = _localStatePrev.r;
    // _nnData.car_acceleration.x = _acceleration.ax;
    // _nnData.car_acceleration.y = _acceleration.ay;
    // _nnData.progress = _progress;
    // _nnData.lmpc_exit_flag = _exit_code;
    // _nnData.dist2centerline = _minDistance;
    // _nnData.index = _index;
    // _nnData.epsilon = _output.z[2];
    // _nnData.pred_acceleration.x = (_localState.vx - _localStatePrev.vx)/(1/_controller_freq) - _localState.vy*_localState.r;
    // _nnData.pred_acceleration.y = (_localState.vy - _localStatePrev.vy)/(1/_controller_freq) + _localState.vx*_localState.r;
}

/**
*	Name: runAlgorithm
*	Description: Calls FORCES solver to solve the MPC problem
*	Inputs: None
*	Output: Void
*/
void LearningMpcc::runAlgorithm()
{
    _start=ros::Time::now();
    
    _prevDash = _currentDash;
    _currentDash = _dash;

    _prevControlCmd = _currentControlCmd;
    _currentControlCmd = _controlCmd;

    _previous_exit_code = _exit_code;

    if (_centerline.poses.size()!= 0){
        buildSpline();
        // ROS_WARN("------SPLINE CONSTRUCTED-------------------------");
        // Assign coeffs
        for (int i = 53; i < _mpcParams.N * (_mpcParams.p + _mpcParams.NN_params + (_mpcParams.spline_points-1)*4*2); i += _mpcParams.p + _mpcParams.NN_params + (_mpcParams.spline_points-1)*4*2)
        {
            _params.all_parameters[i] = _splineLength;
        }
        // X Coeffs
        for (int i = _mpcParams.p + _mpcParams.NN_params; i < _mpcParams.N * (_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2); i += _mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)
        {
            for (int j = 0; j < (_mpcParams.spline_points-1); j++)
            {
                _params.all_parameters[i + j*4] = _aCoeffsX[j];
                _params.all_parameters[i + j*4 + 1] = _bCoeffsX[j];
                _params.all_parameters[i + j*4 + 2] = _cCoeffsX[j];
                _params.all_parameters[i + j*4 + 3] = _dCoeffsX[j];
            }
        }
        // Y Coeffs
        for (int i = _mpcParams.p + _mpcParams.NN_params + (_mpcParams.spline_points-1)*4; i < _mpcParams.N * (_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2); i += _mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2) 
        {
            for (int j = 0; j < (_mpcParams.spline_points-1); j++)
            {
                _params.all_parameters[i + j*4] = _aCoeffsY[j];
                _params.all_parameters[i + j*4 + 1] = _bCoeffsY[j];
                _params.all_parameters[i + j*4 + 2] = _cCoeffsY[j];
                _params.all_parameters[i + j*4 + 3] = _dCoeffsY[j];
            }
        }

        // Assign initial conditions
        _params.xinit[0] = _currentControlCmd.throttle; 
        if (_mpcParams.dash == false){
            _params.xinit[1] = _currentControlCmd.steering_angle;
        }else{
            _params.xinit[1] = -(((_currentDash.steering_angle/10)*PI)/180)/_mpcParams.dash_factor;
        }
        _params.xinit[2] = _prevControlCmd.throttle; 
        if (_mpcParams.dash == true){
            _params.xinit[3] = -(((_prevDash.steering_angle/10)*PI)/180)/_mpcParams.dash_factor;
        }else{
            _params.xinit[3] = _prevControlCmd.steering_angle;
        }
        _params.xinit[4] = _localStatePrev.vx;
        _params.xinit[5] = _localStatePrev.vy;
        _params.xinit[6] = _localStatePrev.r;
        _params.xinit[7] = 0;
        _params.xinit[8] = _globalState.x;
        _params.xinit[9] = _globalState.y;
        _params.xinit[10] = _globalState.psi;
        _params.xinit[11] = _localState.vx;
        _params.xinit[12] = _localState.vy;
        _params.xinit[13] = _localState.r;

        _exit_code = solverMPCC_solve(&_params, &_output, &_info, _mem,  NULL, _extfunc_eval);
        ROS_WARN("Exit Code: %d     Index: %d/%d    Vel: %f",_exit_code,_index,_centerline.poses.size(),std::sqrt(pow(_localState.vx,2) + pow(_localState.vy,2)));

        // Decide on the control actions to apply
        if (_exit_code == 1)
        {
            // _controlCmd = _purePursuit.getControlCmd();
            _controlCmd.throttle = _output.z[3 + _mpcParams.z];
            _controlCmd.steering_angle = _output.z[4 + _mpcParams.z];

            _throttleViz.data = _controlCmd.throttle;
            _steeringAngleViz.data = _controlCmd.steering_angle;

            _fail_counter = 0;
        }
        else if (_fail_counter<=_safe_ctrl_actions.cols())
        {
            // _controlCmd = _purePursuit.getControlCmd();
            _controlCmd.throttle = _safe_ctrl_actions(0, _fail_counter);
            _controlCmd.steering_angle = _safe_ctrl_actions(1, _fail_counter);
            _fail_counter++;
        }else{
            _controlCmd = _purePursuit.getControlCmd();
            // _controlCmd.throttle = 0.1; 
        }
    }

    _CPUtime.data =ros::Time::now().toSec() - _start.toSec();

    // Updata horizon debug variable
    update_horizon();

    // MPC NN DATA
    _nnData.header.stamp = _mpcInfo.header.stamp;
    _nnData.control_cmd = _currentControlCmd;
    _nnData.prev_control_cmd = _prevControlCmd;
    _nnData.car_pose.x = _globalState.x;
    _nnData.car_pose.y = _globalState.y;
    _nnData.car_pose.theta = _globalState.psi;
    _nnData.car_velocity.velocity.x = _localState.vx;
    _nnData.car_velocity.velocity.y = _localState.vy;
    _nnData.car_velocity.velocity.theta = _localState.r;
    _nnData.prev_car_velocity.velocity.x = _localStatePrev.vx;
    _nnData.prev_car_velocity.velocity.y = _localStatePrev.vy;
    _nnData.prev_car_velocity.velocity.theta = _localStatePrev.r;
    _nnData.car_acceleration.x = _acceleration.ax;
    _nnData.car_acceleration.y = _acceleration.ay;
    _nnData.progress = _progress;
    _nnData.lmpc_exit_flag = _exit_code;
    _nnData.dist2centerline = _minDistance;
    _nnData.index = _index;
    _nnData.epsilon = _output.z[2];
    _nnData.pred_acceleration.x = (_localState.vx - _localStatePrev.vx)/(1/_controller_freq) - _localState.vy*_localState.r;
    _nnData.pred_acceleration.y = (_localState.vy - _localStatePrev.vy)/(1/_controller_freq) + _localState.vx*_localState.r;
    _nnData.mpc_output = _horizon;
}
/**
*	Name: getYawFromQuaternion.
*	Description: Converts quaternion matrix in yaw angle.
*	Inputs: Odometry message of the type nav_msgs::Odometry
*	Output: void
*/
double LearningMpcc::getYawFromQuaternion(const nav_msgs::Odometry &pose)
{
    double roll, pitch, yaw;

    tf::Quaternion q(
        pose.pose.pose.orientation.x,
        pose.pose.pose.orientation.y,
        pose.pose.pose.orientation.z,
        pose.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
    return yaw;
}

/**
*	Name: setMarkerArray.
*	Description: Writes debug information on the console
*	Inputs: None
*	Output: Void
*/
void LearningMpcc::lmpcInfoDebug()
{
    // MPC INFO
    _mpcInfo.header.stamp = ros::Time::now();
    _mpcInfo.solve_time = _info.solvetime;
    _mpcInfo.iteration = _info.it;
    _mpcInfo.exit_code = _exit_code;
    _mpcInfo.velocity = std::sqrt(pow(_localState.vx,2) + pow(_localState.vy,2));
    _mpcInfo.throttle = _controlCmd.throttle;
    _mpcInfo.steering = _controlCmd.steering_angle;
    _mpcInfo.progress = _progress;

    // MPC model mismatch
    _modelInfo.header.stamp = _mpcInfo.header.stamp;

    _X_real(0) = _globalState.x;
    _X_real(1) = _globalState.y;
    _X_real(2) = _globalState.psi;
    _X_real(3) = _localState.vx;
    _X_real(4) = _localState.vy;
    _X_real(5) = _localState.r;

    _realModel.X = _X_real(0);
    _realModel.Y = _X_real(1);
    _realModel.Psi = _X_real(2);
    _realModel.vx = _X_real(3);
    _realModel.vy = _X_real(4);
    _realModel.r = _X_real(5);

    _modelInfo.X_real = _realModel;
    
    if (_previous_exit_code == 1)
    {
        _X_diff = _X_real - _X_blended;

        _blendedModel.X = _X_blended(0);
        _blendedModel.Y = _X_blended(1);
        _blendedModel.Psi = _X_blended(2);
        _blendedModel.vx = _X_blended(3);
        _blendedModel.vy = _X_blended(4);
        _blendedModel.r = _X_blended(5);

        _diffModel.X = _X_diff(0);
        _diffModel.Y = _X_diff(1);
        _diffModel.Psi = _X_diff(2);
        _diffModel.vx = _X_diff(3);
        _diffModel.vy = _X_diff(4);
        _diffModel.r = _X_diff(5);

        _modelInfo.X_blended = _blendedModel;
        _modelInfo.X_diff = _diffModel;
    }

    if (_exit_code == 1)
    {
        for (int i = 0; i < _X_blended.size(); i++) // update blended
        {
            _X_blended(i) = _output.z[i + 11 + _mpcParams.z];
        }

    // MPC horizons all states
        _outputDebug.header.stamp = _mpcInfo.header.stamp;
        _outputZI.epsilon = 0.0;
        _outputZI.d_t = _output.zN_1[3];
        _outputZI.delta_t = _output.zN_1[4];
        // _outputZI.d_t_1 = _output.zN_1[2];
        // _outputZI.delta_t_1 = _output.zN_1[3];
        // _outputZI.s_t = _output.zN_1[4];
        // _outputZI.X = _output.zN_1[5];
        // _outputZI.Y = _output.zN_1[6];
        // _outputZI.Psi = _output.zN_1[7];
        // _outputZI.vx = _output.zN_1[8];
        // _outputZI.vy = _output.zN_1[9];
        // _outputZI.r = _output.zN_1[10];
        _outputDebug.first_output = _outputZI;
        _outputDebug.mid_outputs.clear();
        for (int i = 0; i < _mpcParams.N - 2; i++)
        {
            _outputZ.epsilon = 0.0;
            _outputZ.d_t = _output.z[i * _mpcParams.z + 3];
            _outputZ.delta_t = _output.z[i * _mpcParams.z + 4];
            // _outputZ.d_t_1 = _output.z[i * _mpcParams.z + 2];
            // _outputZ.delta_t_1 = _output.z[i * _mpcParams.z + 3];
            // _outputZ.s_t_1 = 0.0;
            // _outputZ.s_t = _output.z[i * _mpcParams.z + 4];
            // _outputZ.X = _output.z[i * _mpcParams.z + 5];
            // _outputZ.Y = _output.z[i * _mpcParams.z + 6];
            // _outputZ.Psi = _output.z[i * _mpcParams.z + 7];
            // _outputZ.vx = _output.z[i * _mpcParams.z + 8];
            // _outputZ.vy = _output.z[i * _mpcParams.z + 9];
            // _outputZ.r = _output.z[i * _mpcParams.z + 10];
            _outputDebug.mid_outputs.push_back(_outputZ);
        }
        // _outputZN.epsilon = 0.0;
        // _outputZN.s_t_1 = 0.0;
        // _outputZN.s_t = _output.zN[0];
        // _outputZN.X = _output.zN[1];
        // _outputZN.Y = _output.zN[2];
        // _outputZN.Psi = _output.zN[3];
        // _outputZN.vx = _output.zN[4];
        // _outputZN.vy = _output.zN[5];
        // _outputZN.r = _output.zN[6];
        // _outputDebug.last_output = _outputZN;

    }


    // DOWNFORCE DISTRIBUTION
    float F_downforce = 0.5 * _mpcParams.rho *_mpcParams.C_l * pow(_localState.vx,2);

    float Ff_downforce = F_downforce * _mpcParams.downforce_front;
    float Fr_downforce = F_downforce * _mpcParams.downforce_rear;
    _normalForces.Ff_downforce = Ff_downforce;
    _normalForces.Fr_downforce = Fr_downforce;

    float Ff_z_static = _mpcParams.m * (_mpcParams.l_r/(_mpcParams.l_f + _mpcParams.l_r)) * _mpcParams.g;
    float Fr_z_static = _mpcParams.m * (_mpcParams.l_f/(_mpcParams.l_f + _mpcParams.l_r)) * _mpcParams.g;

    float Ff_z_no_load = Ff_z_static + Ff_downforce;
    float Fr_z_no_load = Fr_z_static + Fr_downforce;

    // MODEL Blend
    float velocity = sqrt(pow(_localState.vx,2) + pow(_localState.vy,2));
    _modelInfo.lambda = std::min(std::max((sqrt(pow(_localState.vx,2) + pow(_localState.vy,2)) - _mpcParams.lambda_blend_min)/(_mpcParams.lambda_blend_max- _mpcParams.lambda_blend_min), 0.0), 1.0);


    if (velocity >= _mpcParams.lambda_blend_min){
        // TIRE SLIPS
        float alpha_f = _controlCmd.steering_angle - atan2(_localState.vy + _localState.r * _mpcParams.l_f, _localState.vx);
        float alpha_r = atan2(_localState.vy - _localState.r * _mpcParams.l_r, _localState.vx);
        _tireModel.alpha_f = alpha_f;
        _tireModel.alpha_r = alpha_r;

        // LONGITUDINAL LOAD TRANSFER
        bool t_bool_load = (_prevControlCmd.throttle < 0);
        float T_front_load = (!t_bool_load)*_mpcParams.T_max_front + t_bool_load*(_mpcParams.T_brake_front);
        float T_rear_load = (!t_bool_load)*_mpcParams.T_max_rear + t_bool_load*(_mpcParams.T_brake_rear);
    
        float F_drag = 0.5*_mpcParams.rho*_mpcParams.C_d*std::pow(_localState.vx,2);
        float F_roll = _mpcParams.m*_mpcParams.C_roll*_mpcParams.g;

        float Fx_f_load = 2*_mpcParams.eta_motor*T_front_load*_mpcParams.GR/_mpcParams.r_wheel*_prevControlCmd.throttle;
        float Fx_r_load = 2*_mpcParams.eta_motor*T_rear_load*_mpcParams.GR/_mpcParams.r_wheel*_prevControlCmd.throttle;

        float Ff_tire_load = (Ff_z_no_load)*_mpcParams.D*sin(_mpcParams.C*atan(_mpcParams.B*alpha_f));

        float F_x_ax = (Fx_f_load*cos(_prevControlCmd.steering_angle) - Ff_tire_load*sin(_prevControlCmd.steering_angle) + Fx_r_load - F_roll - F_drag);
        float a_x = F_x_ax/_mpcParams.m - _localState.vy*_localState.r;
        _normalForces.pred_ax = a_x;

        float delta_m = _mpcParams.h_cog/(_mpcParams.l_r+_mpcParams.l_f)*_mpcParams.m*a_x/_mpcParams.g;
        float delta_Fz = delta_m*_mpcParams.g;
        _normalForces.Fz_longitudinal_load_transfer = delta_Fz;
        delta_Fz = 0;

        // NEW DOWNFORCE
        delta_Fz = 0;       // REMOVING LOAD TRANSFER
        float Ff_z = Ff_z_no_load - delta_Fz;
        float Fr_z = Fr_z_no_load + delta_Fz;
        _normalForces.Ff_z = Ff_z;
        _normalForces.Fr_z = Fr_z;

        // LATERAL FORCES
        float Ff_y = _mpcParams.D * Ff_z * sin(_mpcParams.C * atan(_mpcParams.B * alpha_f));
        float Fr_y = _mpcParams.D * Fr_z * sin(_mpcParams.C * atan(_mpcParams.B * alpha_r));
        _tireModel.Ff_y = Ff_y;
        _tireModel.Fr_y = Fr_y;

        // LATERAL LOAD TRANSFER
        float F_y_ay = Fx_f_load*sin(_prevControlCmd.steering_angle) + Ff_y*cos(_prevControlCmd.steering_angle) - Fr_y;
        float a_y = F_y_ay/_mpcParams.m + _localState.vx*_localState.r;
        _normalForces.pred_ay = a_y;

        // FRICION ELLIPSE
        if (_modelInfo.lambda == 1 && _exit_code == 1) {

            bool t_bool = (_controlCmd.throttle < 0);
            float T_front = (!t_bool)*_mpcParams.T_max_front + t_bool*(_mpcParams.T_brake_front);
            float T_rear = (!t_bool)*_mpcParams.T_max_rear + t_bool*(_mpcParams.T_brake_rear);

            float Ff_x = 2*_mpcParams.eta_motor*T_front*_mpcParams.GR/_mpcParams.r_wheel*_controlCmd.throttle;
            float Fr_x = 2*_mpcParams.eta_motor*T_rear*_mpcParams.GR/_mpcParams.r_wheel*_controlCmd.throttle;

            // float ellipse_front = pow(Fx_f/(_mpcParams.mu_x*Ff_z),2) + pow(Ff_y/(_mpcParams.mu_y*Ff_z),2);
            // float ellipse_rear = pow(Fx_r/(_mpcParams.mu_x*Fr_z),2) + pow(Fr_y/(_mpcParams.mu_y*Fr_z),2);

            _frictionEllipseFront.ellipse_x_axis.clear();
            _frictionEllipseFront.ellipse_y_axis.clear();
            _frictionEllipseRear.ellipse_x_axis.clear();
            _frictionEllipseRear.ellipse_y_axis.clear();
            // FRONT TIRE
            // Forces
            _frictionEllipseFront.ellipse_x_axis.push_back(Ff_y) ;
            _frictionEllipseFront.ellipse_y_axis.push_back(Ff_x) ;

            // Ellipse max points
            _frictionEllipseFront.ellipse_x_axis.push_back(_mpcParams.mu_y*Ff_z);
            _frictionEllipseFront.ellipse_y_axis.push_back(0);

            _frictionEllipseFront.ellipse_x_axis.push_back(0);
            _frictionEllipseFront.ellipse_y_axis.push_back(_mpcParams.mu_x*Ff_z);

            _frictionEllipseFront.ellipse_x_axis.push_back(-_mpcParams.mu_y*Ff_z);
            _frictionEllipseFront.ellipse_y_axis.push_back(0);

            _frictionEllipseFront.ellipse_x_axis.push_back(0);
            _frictionEllipseFront.ellipse_y_axis.push_back(-_mpcParams.mu_x*Ff_z);

            _frictionEllipseFront.ellipse_x_axis.push_back(Ff_y);
            _frictionEllipseFront.ellipse_y_axis.push_back(_mpcParams.mu_x*std::sqrt(pow(Ff_z,2)-(pow(Ff_y/_mpcParams.mu_y,2))));
        
            _frictionEllipseFront.ellipse_x_axis.push_back(-Ff_y);
            _frictionEllipseFront.ellipse_y_axis.push_back(_mpcParams.mu_x*std::sqrt(pow(Ff_z,2)-(pow(Ff_y/_mpcParams.mu_y,2))));

            _frictionEllipseFront.ellipse_x_axis.push_back(Ff_y);
            _frictionEllipseFront.ellipse_y_axis.push_back(-_mpcParams.mu_x*std::sqrt(pow(Ff_z,2)-(pow(Ff_y/_mpcParams.mu_y,2))));
        
            _frictionEllipseFront.ellipse_x_axis.push_back(-Ff_y);
            _frictionEllipseFront.ellipse_y_axis.push_back(-_mpcParams.mu_x*std::sqrt(pow(Ff_z,2)-(pow(Ff_y/_mpcParams.mu_y,2))));

            _frictionEllipseFront.ellipse_y_axis.push_back(Ff_x);
            _frictionEllipseFront.ellipse_x_axis.push_back(_mpcParams.mu_y*std::sqrt(pow(Ff_z,2)-(pow(Ff_x/_mpcParams.mu_x,2))));
        
            _frictionEllipseFront.ellipse_y_axis.push_back(-Ff_x);
            _frictionEllipseFront.ellipse_x_axis.push_back(_mpcParams.mu_y*std::sqrt(pow(Ff_z,2)-(pow(Ff_x/_mpcParams.mu_x,2))));

            _frictionEllipseFront.ellipse_y_axis.push_back(Ff_x);
            _frictionEllipseFront.ellipse_x_axis.push_back(-_mpcParams.mu_y*std::sqrt(pow(Ff_z,2)-(pow(Ff_x/_mpcParams.mu_x,2))));
        
            _frictionEllipseFront.ellipse_y_axis.push_back(-Ff_x);
            _frictionEllipseFront.ellipse_x_axis.push_back(-_mpcParams.mu_y*std::sqrt(pow(Ff_z,2)-(pow(Ff_x/_mpcParams.mu_x,2))));
            // REAR TIRE
            // Forces
            _frictionEllipseRear.ellipse_x_axis.push_back(Fr_y) ;
            _frictionEllipseRear.ellipse_y_axis.push_back(Fr_x) ;

            // Ellipse max points
            _frictionEllipseRear.ellipse_x_axis.push_back(_mpcParams.mu_y*Fr_z);
            _frictionEllipseRear.ellipse_y_axis.push_back(0);

            _frictionEllipseRear.ellipse_x_axis.push_back(0);
            _frictionEllipseRear.ellipse_y_axis.push_back(_mpcParams.mu_x*Fr_z);

            _frictionEllipseRear.ellipse_x_axis.push_back(-_mpcParams.mu_y*Fr_z);
            _frictionEllipseRear.ellipse_y_axis.push_back(0);

            _frictionEllipseRear.ellipse_x_axis.push_back(0);
            _frictionEllipseRear.ellipse_y_axis.push_back(-_mpcParams.mu_x*Fr_z);

            _frictionEllipseRear.ellipse_x_axis.push_back(Fr_y);
            _frictionEllipseRear.ellipse_y_axis.push_back(_mpcParams.mu_x*std::sqrt(pow(Fr_z,2)-(pow(Fr_y/_mpcParams.mu_y,2))));
        
            _frictionEllipseRear.ellipse_x_axis.push_back(-Fr_y);
            _frictionEllipseRear.ellipse_y_axis.push_back(_mpcParams.mu_x*std::sqrt(pow(Fr_z,2)-(pow(Fr_y/_mpcParams.mu_y,2))));

            _frictionEllipseRear.ellipse_x_axis.push_back(Fr_y);
            _frictionEllipseRear.ellipse_y_axis.push_back(-_mpcParams.mu_x*std::sqrt(pow(Fr_z,2)-(pow(Fr_y/_mpcParams.mu_y,2))));
        
            _frictionEllipseRear.ellipse_x_axis.push_back(-Fr_y);
            _frictionEllipseRear.ellipse_y_axis.push_back(-_mpcParams.mu_x*std::sqrt(pow(Fr_z,2)-(pow(Fr_y/_mpcParams.mu_y,2))));

            _frictionEllipseRear.ellipse_y_axis.push_back(Fr_x);
            _frictionEllipseRear.ellipse_x_axis.push_back(_mpcParams.mu_y*std::sqrt(pow(Fr_z,2)-(pow(Fr_x/_mpcParams.mu_x,2))));
        
            _frictionEllipseRear.ellipse_y_axis.push_back(-Fr_x);
            _frictionEllipseRear.ellipse_x_axis.push_back(_mpcParams.mu_y*std::sqrt(pow(Fr_z,2)-(pow(Fr_x/_mpcParams.mu_x,2))));

            _frictionEllipseRear.ellipse_y_axis.push_back(Fr_x);
            _frictionEllipseRear.ellipse_x_axis.push_back(-_mpcParams.mu_y*std::sqrt(pow(Fr_z,2)-(pow(Fr_x/_mpcParams.mu_x,2))));
        
            _frictionEllipseRear.ellipse_y_axis.push_back(-Fr_x);
            _frictionEllipseRear.ellipse_x_axis.push_back(-_mpcParams.mu_y*std::sqrt(pow(Fr_z,2)-(pow(Fr_x/_mpcParams.mu_x,2))));
        }
    }
    // ROS_WARN("alpha_f: %f   alpha_r: %f    approx_alpha_f: %f   approx_alpha_r: %f", _tireModel.alpha_f, _tireModel.alpha_r, _controlCmd.steering_angle - (_localState.vy + _localState.r * _mpcParams.l_f)/ _localState.vx, (_localState.vy - _localState.r * _mpcParams.l_r)/ _localState.vx);
}
/**
*	Name: setMarkerArray.
*	Description: Sets ROS Marker Array message variable.
*	Inputs: None
*	Output: Void
*/
void LearningMpcc::setMarkerArray()
{
    float scale_vel = 0.1;
    // Prediction from MPC
    _predictions.markers.clear();
    _predictions_text.markers.clear();
    _predictions_velocity.markers.clear();

    int indX = 11;
    int indY = 12;
    int indVelX = 14;
    int indVelY = 15;
    int indProg = 10;
    
    int indXLast = 5;
    int indYLast = 6;
    int indVelXLast = 8;
    int indVelYLast = 9;
    int indProgLast = 4;

    float scaled_vel;

    // In between timesteps
    for (int i = 0; i < (_mpcParams.N-2); i++)
    {
        // Position
        _marker_prediction.pose.position.x = _output.z[i*_mpcParams.z + indX];
        _marker_prediction.pose.position.y = _output.z[i*_mpcParams.z + indY];
        _marker_prediction.scale.x = 0.75f;
        _marker_prediction.id = i + 1;
        // Text
        _marker_prediction_text.pose.position.x = _output.z[i*_mpcParams.z + indX];
        _marker_prediction_text.pose.position.y = _output.z[i*_mpcParams.z + indY];
        _marker_prediction_text.pose.position.z = 0.5;
        _marker_prediction_text.text = "Vel: " + std::to_string(_output.z[i*_mpcParams.z + indVelX]) + " Prog: " + std::to_string(_output.z[i*_mpcParams.z + indProg]);
        _marker_prediction_text.id = i + 1;
        // Velocities
        _marker_velocity.header = _globalState.header;
        _marker_velocity.pose.position.x = _output.z[i*_mpcParams.z + indX];
        _marker_velocity.pose.position.y = _output.z[i*_mpcParams.z + indY];
        scaled_vel = std::sqrt(std::pow(_output.z[i*_mpcParams.z + indVelX],2) + std::pow(_output.z[i*_mpcParams.z + indVelY],2))*scale_vel;
        _marker_velocity.pose.position.z = scaled_vel/2;
        _marker_velocity.scale.z = scaled_vel;
        _marker_velocity.id = i;

        // Push markers into corresponding arrays
        _predictions.markers.push_back(_marker_prediction);
        _predictions_text.markers.push_back(_marker_prediction_text);
        _predictions_velocity.markers.push_back(_marker_velocity);
    }
    // // N-1 timestep
    // Positions
    _marker_prediction.header = _globalState.header;
    _marker_prediction.pose.position.x = _output.zN_1[indX-2];
    _marker_prediction.pose.position.y = _output.zN_1[indY-2];
    _marker_prediction.scale.x = 0.75f;
    _marker_prediction.id = 0;
    // Text
    _marker_prediction_text.header = _globalState.header;
    _marker_prediction_text.pose.position.x = _output.zN_1[indX-2];
    _marker_prediction_text.pose.position.y = _output.zN_1[indY-2];
    _marker_prediction_text.pose.position.z = 0.5;
    _marker_prediction_text.text = "Vel: " + std::to_string(_output.zN_1[indVelX]) + " Prog: " + std::to_string(_output.zN_1[indProg]);
    _marker_prediction_text.id = 0;
    // Velocities
    _marker_velocity.header = _globalState.header;
    _marker_velocity.pose.position.x = _output.zN_1[indX-2];
    _marker_velocity.pose.position.y = _output.zN_1[indY-2];
    scaled_vel = std::sqrt(std::pow(_output.zN_1[indVelX-2],2) + std::pow(_output.zN_1[indVelY-2],2))*scale_vel;
    _marker_velocity.pose.position.z = scaled_vel/2;
    _marker_velocity.scale.z = scaled_vel;
    _marker_velocity.id = _mpcParams.N-2;
    // Push markers into corresponding arrays
    _predictions.markers.push_back(_marker_prediction);
    _predictions_text.markers.push_back(_marker_prediction_text);
    _predictions_velocity.markers.push_back(_marker_velocity);

    // Last Timestep
    // Positions
    _marker_prediction.header = _globalState.header;
    _marker_prediction.pose.position.x = _output.zN[indXLast];
    _marker_prediction.pose.position.y = _output.zN[indYLast];
    _marker_prediction.scale.x = 0.75f;
    _marker_prediction.id = _mpcParams.N;
    // Text
    _marker_prediction_text.header = _globalState.header;
    _marker_prediction_text.pose.position.x = _output.zN[indXLast];
    _marker_prediction_text.pose.position.y = _output.zN[indYLast];
    _marker_prediction_text.pose.position.z = 0.5;
    _marker_prediction_text.text = "Vel: " + std::to_string(_output.zN[indVelXLast]) + " Prog: " + std::to_string(_output.zN[indProgLast]);
    _marker_prediction_text.id = _mpcParams.N;
    // Velocities
    _marker_velocity.header = _globalState.header;
    _marker_velocity.pose.position.x = _output.zN[indXLast];
    _marker_velocity.pose.position.y = _output.zN[indYLast];
    scaled_vel = std::sqrt(std::pow(_output.zN[indVelXLast],2) + std::pow(_output.zN[indVelYLast],2))*scale_vel;
    _marker_velocity.pose.position.z = scaled_vel/2;
    _marker_velocity.scale.z = scaled_vel;
    _marker_velocity.id = _mpcParams.N-1;

    // Push markers into corresponding arrays
    _predictions.markers.push_back(_marker_prediction);
    _predictions_text.markers.push_back(_marker_prediction_text);
    _predictions_velocity.markers.push_back(_marker_velocity);

    // Spline Path
    if (_centerline.poses.size()!= 0)
    {
        _splineMarkersMatlab.markers.clear();

        // With Coeffs
        _marker_spline.color.r = 0.33;
        _marker_spline.color.g = 0.33;
        _marker_spline.color.b = 0.33;

        float s = 0;
        for (int i = 0; i<_mpcParams.spline_points-1; i++){
            s = _splineLength*i/_mpcParams.spline_points + _splineLength/20;

            _marker_spline.header = _globalState.header;
            _marker_spline.header.frame_id = "map";
            
            _marker_spline.pose.position.x = _aCoeffsX[i] + _bCoeffsX[i]*(s - _splineLength*i/_mpcParams.spline_points) + _cCoeffsX[i]*std::pow(s-_splineLength*i/_mpcParams.spline_points,2) + _dCoeffsX[i]*std::pow(s-_splineLength*i/_mpcParams.spline_points,3);
            _marker_spline.pose.position.y = _aCoeffsY[i] + _bCoeffsY[i]*(s - _splineLength*i/_mpcParams.spline_points) + _cCoeffsY[i]*std::pow(s-_splineLength*i/_mpcParams.spline_points,2) + _dCoeffsY[i]*std::pow(s-_splineLength*i/_mpcParams.spline_points,3);

            _marker_spline.id = i;
            _splineMarkersMatlab.markers.push_back(_marker_spline);
        }

        // Plot points from centerline from path planner
    }
}

/**
*	Name: writeDataToFile
*	Description: Writes data on data_lmpc.txt file.
*	Inputs: None
*	Output: Void
*/
void LearningMpcc::writeDataToFile()
{
    if (_write_data_file)
    {
        for (int i = 0; i < 9; i++)
        {
            _data_file << _output.z[i] << " ";
        }
        _data_file << _info.solvetime << " ";
        _data_file << _exit_code << std::endl;
    }
}

/**
*	Name: closeDataFile
*	Description: closes data writing file
*	Inputs: none
*	Output: none
*/
void LearningMpcc::closeDataFile()
{
    if (_write_data_file)
    {
        _data_file.close();
    }
}

/**
*	Name: inspectionControlCmd.
*	Description: Sets the control commands for the inspection mission. 
*	Inputs: None.
*	Output: Control Commands (throttle and steering) .
*/
void LearningMpcc::inspectionControlCmd() {
	
	//const double vel = std::hypot(_currentVelocity.velocity.x, _currentVelocity.velocity.y);
	_controlCmd.header.stamp = ros::Time::now();
	_controlCmd.throttle = 0.9; //static_cast<float>(_accelK * (_minSpeed - vel));
	_controlCmd.steering_angle = _steeringInspection;
	
}

void LearningMpcc::setInspectionSteering(const std_msgs::Float64 &inspectionSteering) {
	_steeringInspection = inspectionSteering.data;
}

void LearningMpcc::setSlamPath(const nav_msgs::Path &slamPath){
    _centerline = slamPath;
    _purePursuit.setPath(slamPath);
}

void LearningMpcc::getNearestIndex(){

	double distance = INFINITY;
    _minDistance = INFINITY;

	// look for the trajectory point that is closest to our current position
	for (int i = _index, x = 0; x < _centerline.poses.size() ; i++, x++) {
		if (i == _centerline.poses.size()) i = 0;
		double dx = _globalState.x - _centerline.poses[i].pose.position.x;
		double dy = _globalState.y - _centerline.poses[i].pose.position.y;
		distance = hypot(dx, dy);
		
		// 3 is track width
		if (distance < _minDistance) {
			_index = i;
			_minDistance = distance;
		}
		// if we are going away from the closest point, stop searching
		else if(distance > _minDistance + 1){
			break;
		}
	}
}

void LearningMpcc::buildSpline(){
    double start = ros::Time::now().toSec(); 

    getNearestIndex();

    std::vector<double> X;
    std::vector<double> Y; 
    std::vector<double> tmp_progresses; 

    // Build Spline
    if ((_mission_config ==  "autocross" && !_loopClosure) || _mission_config ==  "acceleration" || _mission_config ==  "skidpad" || (_mission_config == "trackdrive" && !_loadMap && !_loopClosure)){
        // ROS_WARN("MISSION: %s",_mission_config.c_str());
        nav_msgs::Path _newCenterline;
        int numElements = _centerline.poses.size() - _index;
        _newCenterline.poses.resize(numElements);
        std::copy(_centerline.poses.begin() + _index, _centerline.poses.end(), _newCenterline.poses.begin());

        size_t size = std::floor(_newCenterline.poses.size()/2);
        X.resize(size);
        Y.resize(size);
        tmp_progresses.resize(size);

        for(int i=0; i<size; i++){
            X[i] = _newCenterline.poses[i*2].pose.position.x;
            Y[i] = _newCenterline.poses[i*2].pose.position.y;

            if (i==0){
                _splineLength = 0;
            }else{
                _splineLength = _splineLength + std::sqrt(std::pow(X[i]-X[i-1],2)+std::pow(Y[i]-Y[i-1],2));
            }
            tmp_progresses[i] = _splineLength;
        }
    }else if (_mission_config ==  "trackdrive" || (_mission_config ==  "autocross" && _loopClosure)){
        float max_distance = 50;
        int i = 0;
        int index;
        
        while (true){
            index = _index + i*2;
            if (index > _centerline.poses.size() - 1){
                index = index - _centerline.poses.size() - 1;
            }
            X.push_back(_centerline.poses[index].pose.position.x);
            Y.push_back(_centerline.poses[index].pose.position.y);

            if (i==0){
                _splineLength = 0;
            }else{
                _splineLength = _splineLength + std::sqrt(std::pow(X[i]-X[i-1],2)+std::pow(Y[i]-Y[i-1],2));
            }
            tmp_progresses.push_back(_splineLength);

            i++;

            if(_splineLength > max_distance){
                break;
            }
        }
    }else{
        ROS_ERROR("Mission not found!!!!!!!!");
    }

    tk::spline s_x_tmp(tmp_progresses, X);
    tk::spline s_y_tmp(tmp_progresses, Y);

    std::vector<double> progresses = linspace(0.0, _splineLength, _mpcParams.spline_points);
    std::vector<double> Xs(_mpcParams.spline_points);
    std::vector<double> Ys(_mpcParams.spline_points);

    for (int i=0;i<_mpcParams.spline_points;i++){
        Xs[i] = s_x_tmp(progresses[i]);
        Ys[i] = s_y_tmp(progresses[i]);
    }

    tk::spline s_x(progresses, Xs);
    tk::spline s_y(progresses, Ys);

    Xs.pop_back();
    Ys.pop_back();

    _aCoeffsX = Xs;
    _bCoeffsX = s_x.get_m_b();
    _cCoeffsX = s_x.get_m_c();
    _dCoeffsX = s_x.get_m_d();
    _aCoeffsY = Ys;
    _bCoeffsY = s_y.get_m_b();
    _cCoeffsY = s_y.get_m_c();
    _dCoeffsY = s_y.get_m_d();

    _splineCoeffs.aCoeffsX.clear();
    _splineCoeffs.bCoeffsX.clear();
    _splineCoeffs.cCoeffsX.clear();
    _splineCoeffs.dCoeffsX.clear();
    _splineCoeffs.aCoeffsY.clear();
    _splineCoeffs.bCoeffsY.clear();
    _splineCoeffs.cCoeffsY.clear();
    _splineCoeffs.dCoeffsY.clear();

    _splineCoeffs.aCoeffsX.assign(_aCoeffsX.begin(),_aCoeffsX.end());
    _splineCoeffs.bCoeffsX.assign(_bCoeffsX.begin(),_bCoeffsX.end());
    _splineCoeffs.cCoeffsX.assign(_cCoeffsX.begin(),_cCoeffsX.end());
    _splineCoeffs.dCoeffsX.assign(_dCoeffsX.begin(),_dCoeffsX.end());
    _splineCoeffs.aCoeffsY.assign(_aCoeffsY.begin(),_aCoeffsY.end());
    _splineCoeffs.bCoeffsY.assign(_bCoeffsY.begin(),_bCoeffsY.end());
    _splineCoeffs.cCoeffsY.assign(_cCoeffsY.begin(),_cCoeffsY.end());
    _splineCoeffs.dCoeffsY.assign(_dCoeffsY.begin(),_dCoeffsY.end());
    _splineCoeffs.splineLength = _splineLength;

    double spline_time = ros::Time::now().toSec() - _start.toSec();
    // ROS_WARN("Time For Builond Spline : %lf", spline_time);
}

void LearningMpcc::setNNparams(const std_msgs::Float32MultiArray &msg){
    ROS_WARN("Setting new NN parameters in LMPC");
    for (size_t it = 0; it < _mpcParams.N; it++) {
        for (size_t i = 0; i < msg.data.size(); i++) {
            _params.all_parameters[i+_mpcParams.p+it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2)] = msg.data[i];
        }
        // Activate NN if not
        _params.all_parameters[it*(_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2) + 48] = 1.0f; // Activate NN
        // _params.all_parameters[it*(_mpcParams.p+_mpcParams.NN_params)] = _mpcParams.e_CL_after_train; // Increase centerline weight
    }
    _nnData.just_trained = true;
}

void LearningMpcc::reset_trained_flag(){
    _nnData.just_trained = false;
}

void LearningMpcc::setMPCparams(learning_mpcc::MPCParamsConfig &config){
    // Edit new MPC Params
    _mpcParams.alpha_CL = config.alpha_CL;
    _mpcParams.alpha_L = config.alpha_L;
    _mpcParams.q_r = config.q_r;
    _mpcParams.e_CL_exp = config.e_CL_exp;
    _mpcParams.d_max = config.d_max;
    _mpcParams.beta_steer = config.beta_steer;
    _mpcParams.lambda = config.lambda;
    _mpcParams.v_max = config.v_max;
    _mpcParams.q_v_max = config.q_v_max;
    _mpcParams.l_f = config.l_f;
    _mpcParams.l_r = config.l_r;
    _mpcParams.m = config.m;
    _mpcParams.I_z = config.I_z;
    _mpcParams.T_max_front = config.T_max_front;
    _mpcParams.T_max_rear = config.T_max_rear;
    _mpcParams.T_brake_front = config.T_brake_front;
    _mpcParams.T_brake_rear = config.T_brake_rear;
    _mpcParams.GR = config.GR;
    _mpcParams.eta_motor = config.eta_motor;
    _mpcParams.r_wheel = config.r_wheel;
    _mpcParams.g = config.g;
    _mpcParams.C_roll = config.C_roll;
    _mpcParams.rho = config.rho;
    _mpcParams.lambda_blend_min = config.lambda_blend_min;
    _mpcParams.lambda_blend_max = config.lambda_blend_max;
    _mpcParams.C_d = config.C_d;
    _mpcParams.C_l = config.C_l;
    _mpcParams.B = config.B;
    _mpcParams.C = config.C;
    _mpcParams.D = config.D;
    _mpcParams.downforce_front = config.downforce_front;
    _mpcParams.downforce_rear = config.downforce_rear;
    _mpcParams.beta_throttle = config.beta_throttle;
    _mpcParams.h_cog = config.h_cog;
    _mpcParams.mu_x = config.mu_x;
    _mpcParams.mu_y = config.mu_y;
    _mpcParams.e_CL_max = config.e_CL_max;
    _mpcParams.delta_s_max  = config.delta_s_max;
    _mpcParams.throttle_max = config.throttle_max;
    _mpcParams.steering_max = config.steering_max;
    _mpcParams.delta_throttle_max = config.delta_throttle_max;
    _mpcParams.delta_steering_max = config.delta_steering_max;
    _mpcParams.width = config.width;
    _mpcParams.length = config.length;
    _mpcParams.safety_margin = config.safety_margin;
    _mpcParams.beta_epsilon = config.beta_epsilon;
    _mpcParams.NN_flag = config.NN_flag;
    _mpcParams.beta_psi = config.beta_psi;
    _mpcParams.track_width = config.track_width;
    _mpcParams.diff_gain = config.diff_gain;

    _diffGain.data = _mpcParams.diff_gain;

    // Place Parameters in MPC Solver
    for (int i = 0; i < _mpcParams.N * (_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2); i += (_mpcParams.p+_mpcParams.NN_params+(_mpcParams.spline_points-1)*4*2))
    {
        _params.all_parameters[i] = _mpcParams.alpha_CL;
        _params.all_parameters[i+1] = _mpcParams.alpha_L;
        _params.all_parameters[i+2] = _mpcParams.q_r;
        _params.all_parameters[i+3] = _mpcParams.e_CL_exp;
        _params.all_parameters[i+4] = _mpcParams.d_max;
        _params.all_parameters[i+5] = _mpcParams.beta_steer;
        _params.all_parameters[i+6] = _mpcParams.lambda;
        _params.all_parameters[i+7] = _mpcParams.v_max;
        _params.all_parameters[i+8] = _mpcParams.q_v_max;
        _params.all_parameters[i+9] = _mpcParams.l_f;
        _params.all_parameters[i+10] = _mpcParams.l_r;
        _params.all_parameters[i+11] = _mpcParams.m;
        _params.all_parameters[i+12] = _mpcParams.I_z;
        _params.all_parameters[i+13] = _mpcParams.T_max_front;
        _params.all_parameters[i+14] = _mpcParams.T_max_rear;
        _params.all_parameters[i+15] = _mpcParams.T_brake_front;
        _params.all_parameters[i+16] = _mpcParams.T_brake_rear;
        _params.all_parameters[i+17] = _mpcParams.GR;
        _params.all_parameters[i+18] = _mpcParams.eta_motor;
        _params.all_parameters[i+19] = _mpcParams.r_wheel;
        _params.all_parameters[i+20] = _mpcParams.g;
        _params.all_parameters[i+21] = _mpcParams.C_roll;
        _params.all_parameters[i+22] = _mpcParams.rho;
        _params.all_parameters[i+23] = _mpcParams.lambda_blend_min;
        _params.all_parameters[i+24] = _mpcParams.lambda_blend_max;
        _params.all_parameters[i+25] = _mpcParams.C_d;
        _params.all_parameters[i+26] = _mpcParams.C_l;
        _params.all_parameters[i+27] = _mpcParams.B;
        _params.all_parameters[i+28] = _mpcParams.C;
        _params.all_parameters[i+29] = _mpcParams.D;
        _params.all_parameters[i+30] = _mpcParams.downforce_front;
        _params.all_parameters[i+31] = _mpcParams.downforce_rear;
        _params.all_parameters[i+32] = _mpcParams.beta_throttle;
        _params.all_parameters[i+33] = _mpcParams.h_cog;
        _params.all_parameters[i+34] = _mpcParams.mu_x;
        _params.all_parameters[i+35] = _mpcParams.mu_y;
        _params.all_parameters[i+36] = _mpcParams.e_CL_max;
        _params.all_parameters[i+37] = _mpcParams.delta_s_max;
        _params.all_parameters[i+38] = _mpcParams.throttle_max;
        _params.all_parameters[i+39] = _mpcParams.steering_max;
        _params.all_parameters[i+40] = _mpcParams.delta_throttle_max; 
        _params.all_parameters[i+41] = _mpcParams.delta_steering_max;
        _params.all_parameters[i+42] = _mpcParams.width;
        _params.all_parameters[i+43] = _mpcParams.length;
        _params.all_parameters[i+44] = _mpcParams.safety_margin;
        _params.all_parameters[i+45] = _mpcParams.beta_epsilon;
        _params.all_parameters[i+48] = _mpcParams.NN_flag;
        _params.all_parameters[i+50] = _mpcParams.beta_psi;
        _params.all_parameters[i+51] = _mpcParams.track_width;
        _params.all_parameters[i+52] = _mpcParams.diff_gain;
        _params.all_parameters[i+53] = _splineLength;
    }
}

std::vector<double> linspace(double start, double end, int numPoints) {
    std::vector<double> result(numPoints);
    float stepSize = (end - start) / (numPoints - 1);

    for (int i = 0; i < numPoints; ++i) {
        result[i] = start + stepSize * i;
    }

    return result;
}

void LearningMpcc::update_horizon(){
    _horizon.header.stamp = ros::Time::now();
    _horizon.steps.clear();

    common_msgs::HorizonStep step;

    for (size_t i = 0; i < _mpcParams.N-2; i++)
    {
        step.d_d = _output.z[i*_mpcParams.z];
        step.d_delta = _output.z[1 + i*_mpcParams.z];
        step.epsilon = _output.z[2 + i*_mpcParams.z];
        step.d_t = _output.z[3 + i*_mpcParams.z];
        step.delta_t = _output.z[4 + i*_mpcParams.z];
        step.d_t_1 = _output.z[5+ i*_mpcParams.z];
        step.delta_t_1 = _output.z[6 + i*_mpcParams.z];
        step.vx_t_1 = _output.z[7 + i*_mpcParams.z];
        step.vy_t_1 = _output.z[8 + i*_mpcParams.z];
        step.r_t_1 = _output.z[9 + i*_mpcParams.z];
        step.s_t = _output.z[10 + i*_mpcParams.z];
        step.X = _output.z[11 + i*_mpcParams.z];
        step.Y = _output.z[12 + i*_mpcParams.z];
        step.Psi = _output.z[13 + i*_mpcParams.z];
        step.vx = _output.z[14 + i*_mpcParams.z];
        step.vy = _output.z[15 + i*_mpcParams.z];
        step.r = _output.z[16 + i*_mpcParams.z];

        _horizon.steps.push_back(step);
    }
    
    step.d_d = -420.0;
    step.d_delta = -420.0;
    step.epsilon = _output.zN_1[0];
    step.d_t = _output.zN_1[1];
    step.delta_t = _output.zN_1[2];
    step.d_t_1 = _output.zN_1[3];
    step.delta_t_1 = _output.zN_1[4];
    step.vx_t_1 = _output.zN_1[5];
    step.vy_t_1 = _output.zN_1[6];
    step.r_t_1 = _output.zN_1[7];
    step.s_t = _output.zN_1[8];
    step.X = _output.zN_1[9];
    step.Y = _output.zN_1[10];
    step.Psi = _output.zN_1[11];
    step.vx = _output.zN_1[12];
    step.vy = _output.zN_1[13];
    step.r = _output.zN_1[14];

    _horizon.steps.push_back(step);

    step.d_d = -420.0;
    step.d_delta = -420.0;
    step.epsilon = _output.zN[0];
    step.d_t = -420.0;
    step.delta_t = -420.0;
    step.d_t_1 = -420.0;
    step.delta_t_1 = -420.0;
    step.vx_t_1 = _output.zN[1];
    step.vy_t_1 = _output.zN[2];
    step.r_t_1 = _output.zN[3];
    step.s_t = _output.zN[4];
    step.X = _output.zN[5];
    step.Y = _output.zN[6];
    step.Psi = _output.zN[7];
    step.vx = _output.zN[8];
    step.vy = _output.zN[9];
    step.r = _output.zN[10];

    _horizon.steps.push_back(step);


}