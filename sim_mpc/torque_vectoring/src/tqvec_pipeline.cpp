#include "../include/tqvec_pipeline.hpp"
#define PI 3.14159265359

TorqueVectoring::TorqueVectoring(ros::NodeHandle &nodeHandle): _pidspeeds(0.01, 1, 0, 0, 0) {
    ROS_INFO("Constructed TorqueVectoring");

    LoadParameters();

    _slips.data.resize(4);

    for (int i = 0; i <= 3 ; i++)
    {
        PID* slips_pid = new PID((1/_controllerfrequency), _slipsKp, _slipsKi, _slipsKd, _slipsKlim);
        
        _pidslips.push_back(slips_pid);
        
    }

    _pidspeeds.setPIDParams((1/_controllerfrequency), _speedKp,_speedKi, _speedKd, _speedKlim);

    std::cout << "size of _pid vector: " << _pidslips.size() << std::endl;

}

void TorqueVectoring::LoadParameters(){

    if (!ros::param::get("controller/controller_freq", _controllerfrequency)) {
		ROS_WARN_STREAM("Could not load controller/controller_freq. Default value is 100");
		_controllerfrequency = 100;
	}

    if (!ros::param::get("torque_vectoring/TV_freq", _TVfrequency)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/TV_freq. Default value is 100");
		_TVfrequency = 100;
	}

    if (!ros::param::get("torque_vectoring/torque_vectoring_algorithm", _TV_algorithm)) {
        ROS_WARN_STREAM("Could not load torque_vectoring_algorithm. Default value is 1");
        _TV_algorithm = 4;
    }

    if (!ros::param::get("torque_vectoring/velocity_switch", _vswitch)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/velocity_switch. Default value is 3");
		_vswitch = 3;
	}

    if (!ros::param::get("torque_vectoring/pid2/Kp", _slipsKp)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/pid1/Kp. Default value is 0.3");
		_slipsKp = 0.3;
	}

    if (!ros::param::get("torque_vectoring/pid2/Ki", _slipsKi)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/pid1/Ki. Default value is 0.1");
		_slipsKi = 0.1;
	}

    if (!ros::param::get("torque_vectoring/pid2/Kd", _slipsKd)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/pid1/Kd. Default value is 0.0");
		_slipsKd = 0.0;
	}

    if (!ros::param::get("torque_vectoring/pid2/Klim", _slipsKlim)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/pid1/Klim. Default value is 1.0");
		_slipsKlim = 1.0;
	}

    if (!ros::param::get("torque_vectoring/pid1/Kp", _speedKp)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/pid2/Kp. Default value is 0.3");
		_speedKp = 0.3;
	}

    if (!ros::param::get("torque_vectoring/pid1/Ki", _speedKi)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/pid2/Ki. Default value is 0.1");
		_speedKi = 0.1;
	}

    if (!ros::param::get("torque_vectoring/pid1/Kd", _speedKd)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/pid2/Kd. Default value is 0.0");
		_slipsKd = 0.0;
	}

    if (!ros::param::get("torque_vectoring/pid1/Klim", _speedKlim)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/pid2/Klim. Default value is 1.0");
		_speedKlim = 1.0;
	}
    
    if (!ros::param::get("torque_vectoring/torque_n/rear", _mtrParams.min_rear)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/torque_n/rear. Default value is 100");
		_mtrParams.min_rear = 4.2;
	}

    if (!ros::param::get("torque_vectoring/torque_n/front", _mtrParams.min_front)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/torque_n/front. Default value is 100");
		_mtrParams.min_front = 9.8;
	}
    if (!ros::param::get("torque_vectoring/torque_p/rear", _mtrParams.max_rear)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/torque_p/rear. Default value is 100");
		_mtrParams.max_rear = 6.0;
	}

    if (!ros::param::get("torque_vectoring/torque_p/front", _mtrParams.max_front)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/torque_p/front. Default value is 100");
		_mtrParams.max_front = 4.0;
	}

    if (!ros::param::get("torque_vectoring/max_pwr", _mtrParams.max_pwr)) {
		ROS_WARN_STREAM("Could not load torque_vectoring/max_pwr. Default value is 100");
		_mtrParams.max_pwr = 80000;
	}

    if (!ros::param::get("torque_vectoring/max_rpm", _mtrParams.max_rpm)) {
        ROS_WARN_STREAM("Could not load torque_vectoring/max_rpm. Default value is 100");
        _mtrParams.max_rpm = 20000;
    }

    if (!ros::param::get("common/mission_selected", _mission)) {
		ROS_WARN_STREAM("Could not load mission. Trackdrive will be assumed");
		_mission = TRACKDRIVE;
	}

    if (!ros::param::get("torque_vectoring/num", _params.solver_timeout))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _params.num_of_threads = 4.0;
    }

    if (!ros::param::get("torque_vectoring/solver_timeout", _params.solver_timeout))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _params.solver_timeout = 0.01f;
    }

    if (!ros::param::get("torque_vectoring/mpc_params/objective/z", _mpcParams.z))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.z = 10.0;
    }

    if (!ros::param::get("torque_vectoring/mpc_params/objective/p", _mpcParams.p))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.p = 10.0;
    }

    if (!ros::param::get("torque_vectoring/mpc_params/objective/N", _mpcParams.N))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.N = 15.0;
    }

    if (!ros::param::get("torque_vectoring/mpc_params/objective/w_r", _mpcParams.w_r))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.w_r = 10.0;
    }

    if (!ros::param::get("torque_vectoring/mpc_params/objective/w_u", _mpcParams.w_u))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.w_u = 1.0;
    }

    if (!ros::param::get("torque_vectoring/mpc_params/objective/w_m", _mpcParams.w_m))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.w_m = 0.1;
    }

    if (!ros::param::get("torque_vectoring/mpc_params/objective/w_epsilon_1", _mpcParams.w_epsilon_1))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.w_epsilon_1 = 100.0;
    }

    if (!ros::param::get("torque_vectoring/mpc_params/objective/w_epsilon_2", _mpcParams.w_epsilon_2))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.w_epsilon_2 = 100.0;
    }

    if (!ros::param::get("torque_vectoring/mpc_params/ineq_limits/steering_min", _mpcParams.steering_min))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.steering_min = -0.39;
    }

    if (!ros::param::get("torque_vectoring/mpc_params/ineq_limits/steering_max", _mpcParams.steering_max))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.steering_max = 0.39;
    }

    if (!ros::param::get("torque_vectoring/model_params/l_f", _mpcParams.l_f))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.l_f = 0.88;
    }

    if (!ros::param::get("torque_vectoring/model_params/l_r", _mpcParams.l_r))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.l_r = 0.66;
    }

    if (!ros::param::get("torque_vectoring/model_params/m", _mpcParams.m))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.m = 228.0;
    }

    if (!ros::param::get("torque_vectoring/model_params/I_z", _mpcParams.I_z))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.I_z = 220.0;
    }

    if (!ros::param::get("torque_vectoring/model_params/T_max_front", _mpcParams.T_max_front))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.T_max_front = 4.0;
    }

    if (!ros::param::get("torque_vectoring/model_params/T_max_rear", _mpcParams.T_max_rear))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.T_max_rear = 6.0;
    }

    if (!ros::param::get("torque_vectoring/model_params/T_brake_front", _mpcParams.T_brake_front))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.T_brake_front = 9.8;
    }

    if (!ros::param::get("torque_vectoring/model_params/T_brake_rear", _mpcParams.T_brake_rear))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.T_brake_rear = 4.2;
    }

    if (!ros::param::get("torque_vectoring/model_params/GR", _mpcParams.GR))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.GR = 15.21;
    }

    if (!ros::param::get("torque_vectoring/model_params/eta_motor", _mpcParams.eta_motor))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.eta_motor = 0.90;
    }

    if (!ros::param::get("torque_vectoring/model_params/r_wheel", _mpcParams.r_wheel))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.r_wheel = 0.235;
    }

    if (!ros::param::get("torque_vectoring/model_params/g", _mpcParams.g))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.g = 9.81;
    }

    if (!ros::param::get("torque_vectoring/model_params/C_roll", _mpcParams.C_roll))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.C_roll = 0.0;
    }

    if (!ros::param::get("torque_vectoring/model_params/rho", _mpcParams.rho))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.rho = 1.225;
    }

    if (!ros::param::get("torque_vectoring/model_params/C_d", _mpcParams.C_d))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.C_d = 1.78;
    }

    if (!ros::param::get("torque_vectoring/model_params/C_l", _mpcParams.C_l))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.C_l = 4.64;
    }

    if (!ros::param::get("torque_vectoring/model_params/downforce_front", _mpcParams.downforce_front))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.downforce_front = 0.42;
    }

    if (!ros::param::get("torque_vectoring/model_params/downforce_rear", _mpcParams.downforce_rear))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.downforce_rear = 0.58;
    }

    if (!ros::param::get("torque_vectoring/model_params/K_us_ref", _mpcParams.K_us_ref))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.K_us_ref = 0.017;
    }

    if (!ros::param::get("torque_vectoring/model_params/track_width", _mpcParams.track_width))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.track_width = 1.20;
    }

    if (!ros::param::get("torque_vectoring/tyre_params/D", _mpcParams.D))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.D = 1.0;
    }

    if (!ros::param::get("torque_vectoring/tyre_params/C", _mpcParams.C))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.C = 2.0;
    }

    if (!ros::param::get("torque_vectoring/tyre_params/B", _mpcParams.B))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.B = 10.0;
    }

    if (!ros::param::get("torque_vectoring/tyre_params/mu_x", _mpcParams.mu_x))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.mu_x = 1.5;
    }

    if (!ros::param::get("torque_vectoring/tyre_params/mu_y", _mpcParams.mu_y))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.mu_y = 1.5;
    }

    if (!ros::param::get("torque_vectoring/tyre_params/alpha_max", _mpcParams.alpha_max))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.alpha_max = 0.15;
    }

    if (!ros::param::get("torque_vectoring/diffGain", _diffGain))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _diffGain = 0.7;
    }

    if (!ros::param::get("torque_vectoring/diffGain_acc", _diffGain_acc))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _diffGain_acc = -2.7;
    }

    if (!ros::param::get("torque_vectoring/mpc_params/objective/k_yaw_ref", _mpcParams.k_yaw_ref))
    {
        ROS_WARN("Torque Vectoring: solver timeout not provided. Using 10 ms timeout by default.");
        _mpcParams.k_yaw_ref = 1.0;
    }

    if (!ros::param::get("torque_vectoring/startingRatio", _startingRatio)) {
		_startingRatio = 0.3;
	}
	if (!ros::param::get("torque_vectoring/upGain", _upGain)) {
		_upGain = 3;
	}
	if (!ros::param::get("torque_vectoring/downGain", _downGain)) {
		_downGain = 1.3;
	}
	if (!ros::param::get("torque_vectoring/fallVel", _fallVel)) {
		_fallVel = 12.0;
	}
	if (!ros::param::get("torque_vectoring/startVel", _startVel)) {
		_startVel = 0.0;
	}

    if (!ros::param::get("torque_vectoring/tq_lateral", _tqlateral)) {
		_tqlateral = 1;
	}

    if (!ros::param::get("torque_vectoring/tq_lateral_brake", _tqlateralBrake)) {
		_tqlateralBrake = 0;
	}

    if (!ros::param::get("torque_vectoring/tq_curve", _tqCurve)) {
		_tqCurve = 1;
	}

    if (!ros::param::get("torque_vectoring/tq_curve_brake", _tqCurveBrake)) {
		_tqCurveBrake = 1;
	}


    for (int i = 0; i < _mpcParams.N * _mpcParams.p; i += _mpcParams.p)
    {
        _params.all_parameters[i] = _mpcParams.T_max_front;
        _params.all_parameters[i+1] = _mpcParams.T_max_rear;
        _params.all_parameters[i+2] = _mpcParams.T_brake_front;
        _params.all_parameters[i+3] = _mpcParams.T_brake_rear;
        // throttle
        _params.all_parameters[i+5] = _mpcParams.K_us_ref;
        _params.all_parameters[i+6] = _mpcParams.l_f;
        _params.all_parameters[i+7] = _mpcParams.l_r;
        _params.all_parameters[i+8] = _mpcParams.I_z;
        // Fy_fr_lin
        // Fy_fl_lin
        // Fy_rr_lin
        // Fy_rl_lin
        // alpha_lin_f
        // alpha_lin_r
        // C_alpha_lin_f
        // C_alpha_lin_r 
        _params.all_parameters[i+17] = _mpcParams.r_wheel;
        _params.all_parameters[i+18] = _mpcParams.track_width;
        _params.all_parameters[i+19] = _mpcParams.l_f + _mpcParams.l_r;
        _params.all_parameters[i+20] = _mpcParams.m;
        // v_x
        _params.all_parameters[i+22] = _mpcParams.w_u;
        _params.all_parameters[i+23] = _mpcParams.w_m;
        _params.all_parameters[i+24] = _mpcParams.w_r;
        _params.all_parameters[i+25] = _mpcParams.w_epsilon_1;
        _params.all_parameters[i+26] = _mpcParams.w_epsilon_2;
        _params.all_parameters[i+27] = _mpcParams.GR;
        _params.all_parameters[i+28] = _mpcParams.eta_motor;
        // angular velocity
        _params.all_parameters[i+30] = _mpcParams.mu_x;
        _params.all_parameters[i+31] = _mpcParams.mu_y;
        _params.all_parameters[i+32] = _mpcParams.alpha_max;
        // Fz_fr_lin
        // Fz_fl_lin
        // Fz_rr_lin
        // Fz_rl_lin 
        _params.all_parameters[i+37] = _mpcParams.g;
        _params.all_parameters[i+38] = _mpcParams.N;
        // T_front_ref
        // T_rear_ref
        _params.all_parameters[i+41] = _mpcParams.k_yaw_ref;
    }

    // for (int i = 0; i < _mpcParams.p;i++)
    // {
    //     ROS_WARN("Parameter %d: %f", i, _params.all_parameters[i]);
    // }

    _torques.reserve(4);    

}

/**************************** Getters ****************************/

common_msgs::CarMotor const &TorqueVectoring::getTorques() const {return _torquesMsg;};
common_msgs::CarMotorFloat64 const &TorqueVectoring::getSlips() const {return _slips; };
common_msgs::CarMotor const &TorqueVectoring::getOutputPID_slips() const {return _outputPIDslips; };
common_msgs::CarMotor const &TorqueVectoring::getOutputPID_speeds() const {return _outputPIDspeeds; };
std_msgs::Float64 const &TorqueVectoring::getSpeedError() const {return _error; };
std_msgs::Float64 const &TorqueVectoring::getYawRateRef() const {return _yawRateRef; };
common_msgs::MPCInfo const &TorqueVectoring::getMPCInfo() const { return _mpcInfo; }
std_msgs::Float64 const &TorqueVectoring::getCPUTime() const {return _CPUTime; };
std_msgs::Float32MultiArray const &TorqueVectoring::getTorqueDiffs() const {return _torqueDiffs; };


/**************************** Setters ****************************/


void TorqueVectoring::setControlCmd(const common_msgs::ControlCmd &controlCmd){
    _controlCmd = controlCmd;
    _pedal = controlCmd.throttle;
    _steer_prev = _steer;
    _steer = -controlCmd.steering_angle;
}

void TorqueVectoring::changeControllerState(const common_msgs::ControlMonitoring &msg) {

	// ordered by priority
	if (msg.require_braking) {
		if (msg.control_on == false){
            // means that we are vsv braking and I do not want the controller to act
            _fullBrake = true;   // activate this flag to tqvec knows to not distribute torques
        }
        
        _controlState = ControlState::BRAKING_MODE;
		//ROS_INFO("BRAKING REQUIRED!");
	}
	else if (!msg.control_on) {
		_controlState = ControlState::CONTROL_OFF;
		//ROS_INFO("CONTROL IS OFF");
	}
	else if (_mission == INSPECTION) {
		_controlState = ControlState::INSPECTION;
		//ROS_INFO("INSPECTION MODE");
	}
	else {
        _fullBrake = false; // this is to make sure that if I want to turn off vsv, the car will start going as normal
		_controlState = ControlState::CONTROL_ON;
		//ROS_INFO("CONTROL IS ON");
	}
}


void TorqueVectoring::setWheelSpeed(const common_msgs::CarMotor &wheelSpeed){
	_frontSpeedLeft = wheelSpeed.value0;
	_frontSpeedRight = wheelSpeed.value1;
	_rearSpeedLeft = wheelSpeed.value2;
	_rearSpeedRight = wheelSpeed.value3;

    updateSlips();
}

void TorqueVectoring::setCurrentVelocity(const common_msgs::CarVelocity &vel) {
	_currentVelocity = vel;
    updateSlips();
}

void TorqueVectoring::setSlamPose(const nav_msgs::Odometry &slamPose) {
	_carPose = slamPose;
}

void TorqueVectoring::setDiffGain(const std_msgs::Float64 &diffGain) {
    _diffGain = diffGain.data;
}

void TorqueVectoring::setCurrentAccel(const sensor_msgs::Imu &accel) {
    _accel.x = accel.linear_acceleration.x;
    _accel.y = accel.linear_acceleration.y;
}

/*****************************************************************/


void TorqueVectoring::updateSlips(){

    // calculate slip for each wheel
    _slips.data[0] = (1.0 - (_currentVelocity.velocity.x / (_frontSpeedLeft * ((2.0*PI)/60.0)*(1.0/15.21)*0.25)));
    _slips.data[1] = (1.0 - (_currentVelocity.velocity.x / (_frontSpeedRight * ((2.0*PI)/60.0)*(1.0/15.21)*0.25)));
    _slips.data[2] = (1.0 - (_currentVelocity.velocity.x / (_rearSpeedLeft * ((2.0*PI)/60.0)*(1.0/15.21)*0.25)));
    _slips.data[3] = (1.0 - (_currentVelocity.velocity.x / (_rearSpeedRight * ((2.0*PI)/60.0)*(1.0/15.21)*0.25)));

    
    // iterate over slips and saturate between -1 and 1
    for (int i = 0; i < _slips.data.size(); i++)
    {
        _slips.data[i] = std::min(std::max(_slips.data[i], -1.0), 1.0);
    }

}

void TorqueVectoring::run(){

    _start = ros::Time::now();

    if (_TV_algorithm == 0){
        noTV();
        // ROS_WARN("NO TV");
    }
    else if (_TV_algorithm == 1){
        differential();
        // ROS_WARN("DIFFERENTIAL");
    }
    else if (_TV_algorithm == 2){
        mpcTV();
        // ROS_WARN("MPC TV");
    }
    else if (_TV_algorithm == 3){
        dataDifferential();
        // ROS_WARN("DATA DIFFERENTIAL");
    }
    else if (_TV_algorithm == 4){
        dataDiffAcc();
    }
    else{
        ROS_ERROR("Torque Vectoring: Invalid method");
    }

    
    switch (_mission)
    {
    case ACCELERATION:
        //ROS_WARN("ACCELERATION");
        //controller();
        break;
    
    default:
        break;
    }

    _torqueDiffs.data.clear();
    float torque_diff =_torquesMsg.value0 - _torquesMsg.value1;
    _torqueDiffs.data.push_back(torque_diff);
    torque_diff =_torquesMsg.value2 - _torquesMsg.value3;
    _torqueDiffs.data.push_back(torque_diff);

    _CPUTime.data =ros::Time::now().toSec() - _start.toSec();
    _torquesMsg.header.stamp = ros::Time::now();
}

void TorqueVectoring::noTV(){
    
    if (_pedal >= 0){
        _torques[0] = _mtrParams.max_front*_pedal;
        _torques[1] = _mtrParams.max_front*_pedal;
        _torques[2] = _mtrParams.max_rear*_pedal ;
        _torques[3] = _mtrParams.max_rear*_pedal ;
    } else {
        _torques[0] = _mtrParams.min_front*_pedal;
        _torques[1] = _mtrParams.min_front*_pedal;
        _torques[2] = _mtrParams.min_rear*_pedal ;
        _torques[3] = _mtrParams.min_rear*_pedal ;
    }

    _torquesMsg.value0 = _torques[0]*1e3;
    _torquesMsg.value1 = _torques[1]*1e3;
    _torquesMsg.value2 = _torques[2]*1e3;
    _torquesMsg.value3 = _torques[3]*1e3;
}

// void TorqueVectoring::differential(){


//     double table_front = front(std::fabs(_steer)*180.0/3.1415*5.0);
//     double table_rear = rear(std::fabs(_steer)*180.0/3.1415*5.0);

//     _torques[0] = _mtrParams.max_front*_pedal - table_front*_diffGain*sign(_steer);
//     _torques[1] = _mtrParams.max_front*_pedal + table_front*_diffGain*sign(_steer);
//     _torques[2] = _mtrParams.max_rear*_pedal - table_rear*_diffGain*sign(_steer);
//     _torques[3] = _mtrParams.max_rear*_pedal + table_rear*_diffGain*sign(_steer);

//     saturator();

//     _torquesMsg.value0 = _torques[0]*1e3;
//     _torquesMsg.value1 = _torques[1]*1e3;
//     _torquesMsg.value2 = _torques[2]*1e3;
//     _torquesMsg.value3 = _torques[3]*1e3;

// }

void TorqueVectoring::differential(){

    double table_front = front(_steer);
    double table_rear = rear(_steer);

    if (_pedal >= 0){
        _torques[0] = _mtrParams.max_front*_pedal + table_front*_diffGain/1000;
        _torques[1] = _mtrParams.max_front*_pedal - table_front*_diffGain/1000;
        _torques[2] = _mtrParams.max_rear*_pedal + table_rear*_diffGain/1000;
        _torques[3] = _mtrParams.max_rear*_pedal - table_rear*_diffGain/1000;
    } else {
        _torques[0] = _mtrParams.min_front*_pedal + table_front*_diffGain/1000;
        _torques[1] = _mtrParams.min_front*_pedal - table_front*_diffGain/1000;
        _torques[2] = _mtrParams.min_rear*_pedal + table_rear*_diffGain/1000;
        _torques[3] = _mtrParams.min_rear*_pedal - table_rear*_diffGain/1000;
    } 

    saturator();

    _torquesMsg.value0 = _torques[0]*1000;
    _torquesMsg.value1 = _torques[1]*1000;
    _torquesMsg.value2 = _torques[2]*1000;
    _torquesMsg.value3 = _torques[3]*1000;
}

void TorqueVectoring::dataDifferential(){

    float ratio = 0.5;
    double diff = datadiff(-_steer,_currentVelocity.velocity.x)*_diffGain;
    ratio = std::max(std::min(_accel.x/9.81, 1.0), -1.0)*0.16;

    if (_pedal >= 0){
        _torques[0] = _mtrParams.max_front*_pedal + diff/2*(0.4-ratio);
        _torques[1] = _mtrParams.max_front*_pedal - diff/2*(0.4-ratio);
        _torques[2] = _mtrParams.max_rear*_pedal + diff/2*(0.6+ratio);
        _torques[3] = _mtrParams.max_rear*_pedal - diff/2*(0.6+ratio);
    } else {
        _torques[0] = _mtrParams.min_front*_pedal + diff/2*(0.4-ratio);
        _torques[1] = _mtrParams.min_front*_pedal - diff/2*(0.4-ratio);
        _torques[2] = _mtrParams.min_rear*_pedal + diff/2*(0.6+ratio);
        _torques[3] = _mtrParams.min_rear*_pedal - diff/2*(0.6+ratio);
    } 
    
    saturator();

    _torquesMsg.value0 = _torques[0]*1000;
    _torquesMsg.value1 = _torques[1]*1000;
    _torquesMsg.value2 = _torques[2]*1000;
    _torquesMsg.value3 = _torques[3]*1000;
}

void TorqueVectoring::dataDiffAcc(){

    // for the rear motor I just want the maximum torque
	// for the front motor I want to change the curve based on the formula and velocity
    if ((_tqCurve == 1 && _controlState != ControlState::BRAKING_MODE) || (_tqCurveBrake == 1 && _controlState == ControlState::BRAKING_MODE)){
        const double vel = std::hypot(_currentVelocity.velocity.x, _currentVelocity.velocity.y); 

        float curve1 =  -1 / ( exp(-_downGain * ( vel - _fallVel )) + 1) + 2 - _startingRatio * 2;
        float curve2 =  1 / ( exp(-_upGain * ( vel - _startVel )) + 1);
        torque_mult = curve1 * curve2 + _startingRatio * 2 - 1;

        if ( torque_mult <= 0.0){
            torque_mult = 0.0;
        } else if(torque_mult > 1.0){
            torque_mult = 1.0;
        }

        float curve1_1 =  -1 / ( exp(-_downGain * ( vel - 30.0 )) + 1) + 2 - _startingRatio * 2;
        float curve2_1 =  1 / ( exp(-_upGain * ( vel - 0.0 )) + 1);
        torque_mult_1 = curve1_1 * curve2_1 + _startingRatio * 2 - 1;

        if ( torque_mult_1 <= 0.0){
            torque_mult_1 = 0.0;
        } else if(torque_mult_1 > 1.0){
            torque_mult_1 = 1.0;
        }
    }else{
        torque_mult = 1.0;
        torque_mult_1 = 1.0;
    }

    if ((_tqlateral == 1 && _controlState != ControlState::BRAKING_MODE) || (_tqlateralBrake == 1 && _controlState == ControlState::BRAKING_MODE)){
        diff = datadiff(_carPose.pose.pose.position.y,_currentVelocity.velocity.x)*_diffGain_acc;
        ratio = std::max(std::min(_accel.x/9.81, 1.0), -1.0)*0.16;
    }else{
        diff = 0.0;
    }

    if (_pedal >= 0){
        _torques[0] = std::max((_mtrParams.max_front*_pedal + diff/2*(0.4-ratio))*torque_mult,0.0);
        _torques[1] = std::max((_mtrParams.max_front*_pedal - diff/2*(0.4-ratio))*torque_mult,0.0);
        _torques[2] = std::max((_mtrParams.max_rear*_pedal + diff/2*(0.6+ratio))*torque_mult_1,0.0);
        _torques[3] = std::max((_mtrParams.max_rear*_pedal - diff/2*(0.6+ratio))*torque_mult_1,0.0);
    } else {
        _torques[0] = std::min((_mtrParams.min_front*_pedal + diff/2*(0.4-ratio))*torque_mult,0.0);
        _torques[1] = std::min((_mtrParams.min_front*_pedal - diff/2*(0.4-ratio))*torque_mult,0.0);
        _torques[2] = std::min((_mtrParams.min_rear*_pedal + diff/2*(0.6+ratio))*torque_mult_1,0.0);
        _torques[3] = std::min((_mtrParams.min_rear*_pedal - diff/2*(0.6+ratio))*torque_mult_1,0.0);
    } 

    // special case when I am vsv braking, i dont want the torque vectoring to actuate
    /*
    if (_fullBrake == true && _controlState == ControlState::BRAKING_MODE && _pedal < 0){
        _torques[0] = _mtrParams.min_front*_pedal;
        _torques[1] = _mtrParams.min_front*_pedal;
        _torques[2] = _mtrParams.min_rear*_pedal;
        _torques[3] = _mtrParams.min_rear*_pedal;
    }
    */
    
    
    saturator();

    _torquesMsg.value0 = _torques[0]*1000;
    _torquesMsg.value1 = _torques[1]*1000;
    _torquesMsg.value2 = _torques[2]*1000;
    _torquesMsg.value3 = _torques[3]*1000;
}

void TorqueVectoring::mpcTV(){

    if (sqrt(pow(_currentVelocity.velocity.x,2) + pow(_currentVelocity.velocity.y,2)) < 3){ // Only start operating at 3 m/s

        // differential();
        dataDifferential();

    }

    else{
        
        getTVParams();

        // Set linearization point and other parameters
        for (int i = 0; i < _mpcParams.N * _mpcParams.p; i += _mpcParams.p)
        {
            _params.all_parameters[i+4] = _pedal; // throttle
            _params.all_parameters[i+9] = _mpcParams.Fy_fr_lin;//  Fy_fr_lin
            _params.all_parameters[i+10] = _mpcParams.Fy_fl_lin;// Fy_fl_lin
            _params.all_parameters[i+11] = _mpcParams.Fy_rr_lin;// Fy_rr_lin
            _params.all_parameters[i+12] = _mpcParams.Fy_rl_lin;// Fy_rl_lin
            _params.all_parameters[i+13] = _mpcParams.alpha_lin_f;// alpha_lin_f
            _params.all_parameters[i+14] = _mpcParams.alpha_lin_r;// alpha_lin_r
            _params.all_parameters[i+15] = _mpcParams.C_alpha_lin_f;// C_alpha_lin_f
            _params.all_parameters[i+16] = _mpcParams.C_alpha_lin_r;// C_alpha_lin_r 
            _params.all_parameters[i+21] = _mpcParams.v_x;// v_x
            _params.all_parameters[i+29] = _mpcParams.angular_velocity;// angular velocity
            _params.all_parameters[i+33] = _mpcParams.Fz_fr_lin;// Fz_fr_lin
            _params.all_parameters[i+34] = _mpcParams.Fz_fl_lin;// Fz_fl_lin
            _params.all_parameters[i+35] = _mpcParams.Fz_rr_lin;// Fz_rr_lin
            _params.all_parameters[i+36] = _mpcParams.Fz_rl_lin;// Fz_rl_lin 
            _params.all_parameters[i+39] = _mpcParams.T_front_ref;// T_front_ref
            _params.all_parameters[i+40] = _mpcParams.T_rear_ref;// T_rear_ref
        }

        /* for (int i = 0; i < _mpcParams.p;i++)
        {
            ROS_WARN("Parameter %d: %f", i, _params.all_parameters[i]);
        } */

        // Variable initialization
        _params.xinit[0] = _mpcParams.Mz_prev; 
        _params.xinit[1] = _steer;
        _params.xinit[2] = _currentVelocity.velocity.theta; // Yaw Rate
        _params.xinit[3] = atan(_currentVelocity.velocity.y/_currentVelocity.velocity.x); // Beta

        _exit_code = solverTV_solve(&_params, &_output, &_info, _mem,  NULL, _extfunc_eval);
        ROS_WARN("TV Exit Code: %d",_exit_code);
        // ROS_WARN("FL: %f, FR: %f, RL: %f, RR: %f", _output.zI[0], _output.zI[1], _output.zI[2], _output.zI[3]);

        if (_exit_code == 1){ // Converting from Nm to mNm
            _torques[0] = _output.zI[0];
            _torques[1] = _output.zI[1];
            _torques[2] = _output.zI[2];
            _torques[3] = _output.zI[3];

            _torquesMsg.value0 = _torques[0]*1e3;
            _torquesMsg.value1 = _torques[1]*1e3;
            _torquesMsg.value2 = _torques[2]*1e3;
            _torquesMsg.value3 = _torques[3]*1e3;
        }
        else{
            differential();
        }

        setYawRateRef();
        setInfoMpc();
    }
}

void TorqueVectoring::getTVParams(){

    // Car States
    float Beta = atan(_currentVelocity.velocity.y/_currentVelocity.velocity.x);
    float delta = _steer;
    float r = _currentVelocity.velocity.theta;
    _mpcParams.v_x = _currentVelocity.velocity.x;

    // Car Parameters
    float a = _mpcParams.l_f;
    float b = _mpcParams.l_r;
    float D = _mpcParams.D;
    float C = _mpcParams.C;
    float B = _mpcParams.B;

    // Torque References
    if (_pedal >= 0.0){
        _mpcParams.T_front_ref = _mpcParams.T_max_front*_pedal;
        _mpcParams.T_rear_ref = _mpcParams.T_max_rear*_pedal;
    }else{
        _mpcParams.T_front_ref = _mpcParams.T_brake_front*_pedal;
        _mpcParams.T_rear_ref = _mpcParams.T_brake_rear*_pedal;
    }

    // Steer Angular Velocity
    float h = 1/_TVfrequency;
    _mpcParams.angular_velocity = (_steer - _steer_prev)/h;
    ROS_WARN("Steer: %f, Steer_prev: %f, Angular Velocity: %f", _steer, _steer_prev, _mpcParams.angular_velocity);

    // Normal Forces (Currently without Load transfer)
    _mpcParams.Fz_fl_lin = _mpcParams.m * _mpcParams.g * (b / (a + b))/2;
    _mpcParams.Fz_fr_lin = _mpcParams.m * _mpcParams.g * (b / (a + b))/2;
    _mpcParams.Fz_rl_lin = _mpcParams.m * _mpcParams.g * (a / (a + b))/2;
    _mpcParams.Fz_rr_lin = _mpcParams.m * _mpcParams.g * (a / (a + b))/2;

    // Liniarizations //

    // Slip Angles
    _mpcParams.alpha_lin_f = Beta - delta + a * r / _mpcParams.v_x;
    _mpcParams.alpha_lin_r = Beta - b * r / _mpcParams.v_x;

    // Tire Forces
    _mpcParams.Fy_fl_lin = -D*_mpcParams.Fz_fl_lin*sin(C*atan(B*_mpcParams.alpha_lin_f));
    _mpcParams.Fy_fr_lin = -D*_mpcParams.Fz_fr_lin*sin(C*atan(B*_mpcParams.alpha_lin_f));
    _mpcParams.Fy_rl_lin = -D*_mpcParams.Fz_rl_lin*sin(C*atan(B*_mpcParams.alpha_lin_r));
    _mpcParams.Fy_rr_lin = -D*_mpcParams.Fz_rr_lin*sin(C*atan(B*_mpcParams.alpha_lin_r));

    // Cornering Stiffnesses
    _mpcParams.C_alpha_lin_f = -(B*C*D*(_mpcParams.Fz_fl_lin+_mpcParams.Fz_fr_lin)*cos(C*atan(B*_mpcParams.alpha_lin_f)))/(std::pow(B,2)*std::pow(_mpcParams.alpha_lin_f,2)+1);
    _mpcParams.C_alpha_lin_r = -(B*C*D*(_mpcParams.Fz_rl_lin+_mpcParams.Fz_rr_lin)*cos(C*atan(B*_mpcParams.alpha_lin_r)))/(std::pow(B,2)*std::pow(_mpcParams.alpha_lin_r,2)+1);

    // Mz_prev
    _mpcParams.Mz_prev = _mpcParams.GR * _mpcParams.eta_motor / _mpcParams.r_wheel * _mpcParams.track_width/2 *(-_torques[0] + _torques[1] - _torques[2] + _torques[3]);
}

void TorqueVectoring::setYawRateRef(){
    // Yaw Rate Reference
    _yawRateRef.data = _mpcParams.v_x * _steer / ((_mpcParams.l_r + _mpcParams.l_f) + _mpcParams.K_us_ref * std::pow(_mpcParams.v_x,2));
}

void TorqueVectoring::setInfoMpc(){
    _mpcInfo.header.stamp = ros::Time::now();
    _mpcInfo.solve_time = _info.solvetime;
    _mpcInfo.iteration = _info.it;
    _mpcInfo.exit_code = _exit_code;
    _mpcInfo.velocity = std::sqrt(pow(_currentVelocity.velocity.x,2) + pow(_currentVelocity.velocity.y,2));
    _mpcInfo.throttle = _controlCmd.throttle;
    _mpcInfo.steering = _controlCmd.steering_angle;
}
    


void TorqueVectoring::saturator(){
    // Front
    for (int i = 0; i <= 1; i++)
    {
        _torques[i] = std::max(std::min(_torques[i], _mtrParams.max_front), -_mtrParams.min_front);
    }
    // Rear
    for (int i = 2; i <= 3; i++)
    {      
        _torques[i] = std::max(std::min(_torques[i], _mtrParams.max_rear), -_mtrParams.min_rear);
    
    }
}

double TorqueVectoring::sign(double x) {
    return (x > 0) - (x < 0);
}

void TorqueVectoring::controller(){ 

    if(_currentVelocity.velocity.x <= _vswitch && _currentVelocity.velocity.x >= 0.5){ // use the wheels speeds way

        float average_front_speed = (_frontSpeedLeft + _frontSpeedRight)/2;
        float average_rear_speed = (_rearSpeedLeft + _rearSpeedRight)/2;

        float difference_speed = (average_front_speed - average_rear_speed)/1e3;

        if(difference_speed <= 0){
            return;
        }else{

            double delta_wheels = _pidspeeds.calculate(0,difference_speed);

            _torques[0] += delta_wheels;
            _torques[1] += delta_wheels;

            _outputPIDspeeds.value0 = delta_wheels*1e3;
            _outputPIDspeeds.value1 = delta_wheels*1e3;
            _outputPIDspeeds.value2 = 0;
            _outputPIDspeeds.value3 = 0;

            saturator();

            _torquesMsg.value0 = _torques[0]*1e3;
            _torquesMsg.value1 = _torques[1]*1e3;
        }

        

    }else if(_currentVelocity.velocity.x > _vswitch){ // use the slips way

        std::vector<double> deltas;
        deltas.resize(4);

        for (int i = 0; i < deltas.size(); i++)
        {
            deltas[i] = _pidslips[i]->calculate(0.20,_slips.data[i]);
            if (deltas[i] > 0){
                deltas[i] = 0;
            }
        }

        std::cout << deltas[0] << std::endl;

        _torques[0] += deltas[0];
        _torques[1] += deltas[1];
        _torques[2] += deltas[2];
        _torques[3] += deltas[3]; 

        _outputPIDslips.value0 = deltas[0]*1e3;
        _outputPIDslips.value1 = deltas[1]*1e3;
        _outputPIDslips.value2 = deltas[2]*1e3;
        _outputPIDslips.value3 = deltas[3]*1e3;

        _outputPIDspeeds.value0 = 0; // just in order to clarify things
        _outputPIDspeeds.value1 = 0;

        saturator();

        _torquesMsg.value0 = _torques[0]*1e3;
        _torquesMsg.value1 = _torques[1]*1e3;
        _torquesMsg.value2 = _torques[2]*1e3;
        _torquesMsg.value3 = _torques[3]*1e3;

    }



}

