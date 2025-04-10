#include"ros/ros.h"
#include <as_lib/common.h>
#include "common_msgs/ControlCmd.h"
#include "common_msgs/ControlMonitoring.h"
#include "common_msgs/CarMotor.h"
#include "common_msgs/CarVelocity.h"
#include "common_msgs/MPCInfo.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "common_msgs/CarMotorFloat64.h"
#include "tqvec_pid.hpp"
#include <algorithm>
#include <math.h>
#include "std_msgs/Float32MultiArray.h"
#include "common_msgs/CarAcceleration.h"
#include "sensor_msgs/Imu.h"

#include "../solverTV/include/solverTV.h"
#include "../solverTV/include/solverTV_memory.h"

enum class ControlState {
  CONTROL_OFF,
  CONTROL_ON,
  BRAKING_MODE,
  INSPECTION
};

struct mpcParams
{
    // Objective Function
    float z;
    float p;
    float N;
    float w_r;
    float w_u;
    float w_m;
    float w_epsilon_1;
    float w_epsilon_2;
    float k_yaw_ref;

    // Constrains
    float steering_min;
    float steering_max;

    // Model Params
    float l_f;
    float l_r;
    float m;
    float I_z;
    float T_max_front;
    float T_max_rear;
    float T_brake_front;
    float T_brake_rear;
    float GR;
    float eta_motor;
    float r_wheel;
    float g;
    float C_roll;
    float rho;
    float C_d;
    float C_l;
    float downforce_front;
    float downforce_rear;
    float K_us_ref;
    float track_width;

    // Tyre Params
    float D;
    float C;
    float B;
    float mu_x;
    float mu_y;
    float alpha_max;

    // Lin Params and others
    float Fy_fr_lin;
    float Fy_fl_lin;
    float Fy_rr_lin;
    float Fy_rl_lin;
    float alpha_lin_f;
    float alpha_lin_r;
    float C_alpha_lin_f;
    float C_alpha_lin_r; 
    float v_x;
    float angular_velocity;
    float Fz_fr_lin;
    float Fz_fl_lin;
    float Fz_rr_lin;
    float Fz_rl_lin;
    float T_front_ref;
    float T_rear_ref;
    float Mz_prev;
};


struct MtrParams{
    double min_front;
    double min_rear;
    double max_front;
    double max_rear;
    double max_rpm;
    double max_pwr;
};

/* AD tool to FORCESPRO interface */
extern "C" solver_int32_default solverTV_adtool2forces(solverTV_float *x,        /* primal vars                                         */
                                 solverTV_float *y,        /* eq. constraint multiplers                           */
                                 solverTV_float *l,        /* ineq. constraint multipliers                        */
                                 solverTV_float *p,        /* parameters                                          */
                                 solverTV_float *f,        /* objective function (scalar)                         */
                                 solverTV_float *nabla_f,  /* gradient of objective function                      */
                                 solverTV_float *c,        /* dynamics                                            */
                                 solverTV_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 solverTV_float *h,        /* inequality constraints                              */
                                 solverTV_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 solverTV_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */);
                                 

class TorqueVectoring{

    public:

        //Constructor
        TorqueVectoring(ros::NodeHandle &nodeHandle);

        void LoadParameters();

        // Getters
        common_msgs::CarMotor const & getTorques() const;
        common_msgs::CarMotorFloat64 const & getSlips() const;
        common_msgs::CarMotor const & getOutputPID_slips() const;
        common_msgs::CarMotor const & getOutputPID_speeds() const;
        std_msgs::Float64 const & getSpeedError() const;
        std_msgs::Float64 const & getYawRateRef() const;
        common_msgs::MPCInfo const & getMPCInfo() const;
        std_msgs::Float64 const & getCPUTime() const;
        std_msgs::Float32MultiArray const & getTorqueDiffs() const;

        // Setters
        void setControlCmd(const common_msgs::ControlCmd &controCmd);
        void setWheelSpeed(const common_msgs::CarMotor &wheelSpeed);
        void setCurrentVelocity(const common_msgs::CarVelocity &vel);
        void setSlamPose(const nav_msgs::Odometry &slamPose);
        void setDiffGain(const std_msgs::Float64 &diffGain);
        void setCurrentAccel(const sensor_msgs::Imu &accel);
        void changeControllerState(const common_msgs::ControlMonitoring &msg);
        
        void setSteer(const common_msgs::ControlCmd &controlCmd);

        void run();

    private:

        ControlState _controlState;
        bool _fullBrake = false;
        
        common_msgs::CarMotor _torquesMsg;
        common_msgs::CarMotorFloat64 _slips;
        common_msgs::CarMotor _outputPIDslips;
        common_msgs::CarMotor _outputPIDspeeds;
        common_msgs::ControlCmd _controlCmd;
        std_msgs::Float64 _error;
        std_msgs::Float32MultiArray _torqueDiffs;
        common_msgs::CarAcceleration _accel;

        ros::Time _start;
        std_msgs::Float64 _CPUTime;
        
        std::vector<double> _torques;

        std::vector<PID*> _pidslips;
        PID _pidspeeds;

        double _controllerfrequency;
        double _TVfrequency;
        double _vswitch;
        double _slipsKp;
        double _slipsKi;
        double _slipsKd;
        double _slipsKlim;
        double _speedKp;
        double _speedKi;
        double _speedKd;
        double _speedKlim;
        double _pedal;
        double _steer = 0.0;
        double _steer_prev = 0.0;
        double _rearSpeedLeft;
        double _frontSpeedLeft;
        double _frontSpeedRight;
        double _rearSpeedRight;
        double _integral;
        double _prevError;
        float torque_mult = 1.0;
        float torque_mult_1 = 1.0;
        float ratio = 0.5;
        double diff = 0.0;

        common_msgs::CarVelocity _currentVelocity;
        nav_msgs::Odometry _carPose;


        // Solver parameters
        solverTV_params _params;
        solverTV_output _output;
        solverTV_info _info;
        solverTV_mem* _mem = solverTV_internal_mem(0);
        solverTV_extfunc _extfunc_eval = &solverTV_adtool2forces;
        int _exit_code = -69;
        int _previous_exit_code = 0;

        // mpcTV vars
        double _prevMZ = 0;
        mpcParams _mpcParams;
        int _TV_algorithm;
        std_msgs::Float64 _yawRateRef;
        common_msgs::MPCInfo _mpcInfo;

        double _diffGain;
        double _diffGain_acc;

        float _startingRatio;
        float _upGain;
        float _downGain;
        float _fallVel;
        float _startVel;
        int _tqlateralBrake;
        int _tqlateral;
        int _tqCurve;
        int _tqCurveBrake;


        int _mission;

        // inline double front(double steer) {return 7.44e-7*pow(steer, 2) + 5.24e-5*steer + 0.000414;}
        // inline double rear(double steer) {return 0.00023*steer + 0.00207;}
        inline double front(double steer) {return 71.0145*pow(steer, 3) + 0*pow(steer, 2) + 8.5638*steer + 0;}
        inline double rear(double steer) {return 32.9071*steer + 0.0;}
        inline double datadiff(double steer, double vx) {return 0.0221*pow(steer, 3) - 1.83*steer*vx - 0.6574*steer;}
        
        void noTV();

        void differential();
        void dataDifferential();
        void dataDiffAcc();
        double pid(double setpoint, double pv, int beginning, int method);
        double sign(double x);
        void saturator();
        void controller();
        void updateSlips();

        void mpcTV();
        void getTVParams();
        void setYawRateRef();
        void setInfoMpc();

        MtrParams _mtrParams;

};