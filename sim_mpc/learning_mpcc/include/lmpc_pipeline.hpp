#ifndef LMPC_PIPELINE_HPP
#define LMPC_PIPELINE_HPP


#include "../solverLMPC/include/solverMPCC.h"
#include "../solverLMPC/include/solverMPCC_memory.h"

#include "common_msgs/Output.h"
#include "common_msgs/OutputZI.h"
#include "common_msgs/OutputZ.h"
#include "common_msgs/OutputZN.h"
#include "common_msgs/ModelInfo.h"
#include "common_msgs/ModelInfoCompare.h"
#include "common_msgs/TireModel.h"
#include "common_msgs/NormalForces.h"
#include "common_msgs/FrictionEllipse.h"
#include "common_msgs/NNData.h"
#include "common_msgs/controllerRate.h"
#include "common_msgs/GSS.h"
#include "common_msgs/Horizon.h"
#include "common_msgs/HorizonStep.h"


#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include "common_msgs/ControlCmd.h"
#include "common_msgs/CarVelocity.h"
#include "common_msgs/CarPose.h"
#include "common_msgs/CarAcceleration.h"
#include "common_msgs/MPCInfo.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "common_msgs/ControlMonitoring.h"
#include "sensor_msgs/Imu.h"
#include <ros/ros.h>
#include "common_msgs/SplineCoeffs.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Bool.h"


#include <cmath>
#include <eigen3/Eigen/Dense>
#include <fstream>

#include "lmpc_pure_pursuit.hpp"
#include "spline.h"

#include <iostream>

#include <torch/torch.h>
#include <torch/script.h>

#include <dynamic_reconfigure/server.h>
#include <learning_mpcc/MPCParamsConfig.h>

#include <cstdlib>

#include "json.hpp"
using json = nlohmann::json;

// for noise testing
#include <random>

#include "std_msgs/Float64.h"

#define TAU 6.28318530718f

enum class ControlState {
  CONTROL_OFF,
  CONTROL_ON,
  BRAKING_MODE,
  INSPECTION
};


struct LocalState
{
    float vx = 0.0;
    float vy = 0.0;
    float r = 0.0;
};

struct GlobalState
{
    float x = 0.0;
    float y = 0.0;
    float psi = 0.0;
    std_msgs::Header header;
};

struct Acceleration
{
    float ax = 0.0;
    float ay = 0.0;
};

struct mpcParams
{
    // Objective Function
    int N;
    int z;
    int p;
    float alpha_CL;   //200
    float alpha_L;   //1000
    float q_r;        // 40
    int e_CL_exp;        // 10
    float d_max;      // 0.3
    float beta_steer; // 400
    float beta_throttle;
    float beta_throttle_trackdrive;
    float lambda;     // 150
    float v_max;
    float v_max_trackdrive;
    float q_v_max;
    float lambda_blend_min;
    float lambda_blend_max;
    float beta_epsilon;
    int NN_params;
    float e_CL_after_train;
    float q_r_max;
    float beta_psi;

    // Tire Params
    float C;
    float D;
    float D_trackdrive;
    float B;
    float mu_x;
    float mu_y;
    float mu_x_trackdrive;
    float mu_y_trackdrive;

    // Car Params
    float l_r;
    float l_f;
    float downforce_front;
    float downforce_rear;
    float g;
    float rho;
    float C_l;
    float m;
    float GR;
    float eta_motor;
    float r_wheel;
    float T_max_front;
    float T_max_rear;
    float T_brake_front;
    float T_brake_rear;
    float C_roll;
    float I_z;
    float C_d;
    float h_cog;
    float length;
    float width;
    float NN_flag;
    float track_width;
    float diff_gain;
    int spline_points;

    // Constraints
    float e_CL_max;
    float delta_s_max;
    float throttle_max;
    float steering_max;
    float delta_throttle_max;
    float delta_steering_max;
    float safety_margin;
    float dash_factor;
    bool dash;
};

struct dataParams
{
    int n_ML_vars;
    int n_ML_params;
    int n1;
    int n2_19;
    int n20;
};

struct carParams
{
    float l_f;   
    float l_r;   
    float m; 
    float I_z;      
    float T_max;
    float T_break;     
    float GR;   
    float eta_motor; 
    float r_wheel;
    float g;      
    float C_roll;
    float rho;    
    float C_d;    
    float A_F;
    float D; 
    float C;
    float B;
};

/* AD tool to FORCESPRO interface */
extern "C" solver_int32_default solverMPCC_adtool2forces(
                                 solverMPCC_float *x,        /* primal vars                                         */
                                 solverMPCC_float *y,        /* eq. constraint multiplers                           */
                                 solverMPCC_float *l,        /* ineq. constraint multipliers                        */
                                 solverMPCC_float *p,        /* parameters                                          */
                                 solverMPCC_float *f,        /* objective function (scalar)                         */
                                 solverMPCC_float *nabla_f,  /* gradient of objective function                      */
                                 solverMPCC_float *c,        /* dynamics                                            */
                                 solverMPCC_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 solverMPCC_float *h,        /* inequality constraints                              */
                                 solverMPCC_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 solverMPCC_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */);


class LearningMpcc
{

public:
    // Constructor
    LearningMpcc();

    // Getters
    common_msgs::ControlCmd const &getControlCommand() const;
    visualization_msgs::MarkerArray const &getPredictions() const;
    visualization_msgs::MarkerArray const &getPredictionsText () const;
    visualization_msgs::MarkerArray const &getPosition() const;
    visualization_msgs::MarkerArray const &getPredictionsVelocities () const;
    std_msgs::Float32MultiArray const &getLearningData() const;
    std_msgs::UInt8MultiArray const &getNewCP() const;
    std_msgs::Float32 const & getThrottle() const;
    std_msgs::Float32 const & getSteeringAngle() const;
    common_msgs::MPCInfo const & getMPCInfo() const;
    common_msgs::ModelInfoCompare const & getMPCModel() const;
    common_msgs::Output const & getOutputMPC() const;
    common_msgs::TireModel const & getTire() const;
    common_msgs::NormalForces const & getNormalForces() const;
    common_msgs::FrictionEllipse const & getFrictionEllipseFront() const;
    common_msgs::FrictionEllipse const & getFrictionEllipseRear() const;
    common_msgs::NNData const & getNNData() const;
    std_msgs::Float64 & getDiffGain();
    visualization_msgs::Marker const & getMarkerPointToFollow() const;
    visualization_msgs::Marker const & getMarkerPointAhead() const;
    common_msgs::SplineCoeffs _splineCoeffs;
    visualization_msgs::MarkerArray const &getSplineMarkersMatlab() const;



    // Setters
    void setControlCmd();
    void setInitialGuess();
    void setCurrentVelocity(const common_msgs::CarVelocity &velMsg);
    void setCurrentAcceleration(const sensor_msgs::Imu &accMsg);
    void setSlamPose(const nav_msgs::Odometry &slamPoseMsg);
    void setLoopClosure(const std_msgs::Bool &loopClosure);
    void setMarkerArray();
    void setMpcParams(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void setMLparams(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void changeLmpcState(const common_msgs::ControlMonitoring &msg);
    void changeControllerState(const common_msgs::ControlMonitoring &msg);
    void setInspectionSteering(const std_msgs::Float64 &inspectionSteering);
    void setSlamPath(const nav_msgs::Path &slamPath);
    void setNNparams(const std_msgs::Float32MultiArray &msg);
    void setMPCparams(learning_mpcc::MPCParamsConfig &config);
    void setDashSteering(const common_msgs::ControlCmd &dash);


    // Function declarations
    void loadTrackdriveParams();
    void lmpcInfoDebug();
    bool updateMLmodel();
    bool checkNewCP();
    void writeDataToFile();
    Eigen::MatrixXf readConesTrackFile(const std::string &file);
    int checkForConeHit();
    double getYawFromQuaternion(const nav_msgs::Odometry &pose);
    float saturation(const float &x, const float &min, const float &max);
    void closeDataFile();
    void inspectionControlCmd();
    void getNearestIndex();
    void buildSpline();
    void reset_trained_flag();
    void update_horizon();

    // RUN ALGORITHM
    void run();
    void runAlgorithm();
    std_msgs::Float64 getCPUtime();

    bool _initParams = true;
    bool _publish=false;

private:
    // coordinates and states
    LocalState _localState;
    GlobalState _globalState;
    LocalState _localStatePrev;
    Acceleration _acceleration;

    // pure pursuit struct
    PurePursuit _purePursuit;

    // MPC parameters
    mpcParams _mpcParams;
    float _controller_freq;
    double _progress = 0.0;
    double _track_length;
    int _index = 0;
    int _model_id;
    int _counterLoop = 0;
    std::string _pathModel;

    // Spline Coeffs
    double _splineLength = 0.0;
    std::vector<double> _aCoeffsX;
    std::vector<double> _aCoeffsY;
    std::vector<double> _bCoeffsX;
    std::vector<double> _bCoeffsY;
    std::vector<double> _cCoeffsX;
    std::vector<double> _cCoeffsY;
    std::vector<double> _dCoeffsX;
    std::vector<double> _dCoeffsY;

    // Solver parameters
    solverMPCC_params _params;
    solverMPCC_output _output;
    solverMPCC_info _info;
    solverMPCC_mem* _mem = solverMPCC_internal_mem(0);
    solverMPCC_extfunc _extfunc_eval = &solverMPCC_adtool2forces;
    int _exit_code = -69;
    int _previous_exit_code = 0;

    // data parameters
    dataParams _dataParams;

    //Car params
    carParams _carParams;

    ControlState _controlState = ControlState::CONTROL_OFF;
    float _steeringInspection;
    nav_msgs::Path _centerline;
    bool _loopClosure = false;
    bool _loadMap;
    float _minDistance;
    std::string _lmpcPath;

    // Messages
    common_msgs::ControlCmd _dash;
    common_msgs::ControlCmd _prevDash;
    common_msgs::ControlCmd _currentDash;
    common_msgs::ControlCmd _currentControlCmd;
    common_msgs::ControlCmd _prevControlCmd;
    common_msgs::ControlCmd _controlCmd;
    std_msgs::Float32MultiArray _learningDataMsg; // msg to publish Learning data
    std_msgs::UInt8MultiArray _newCPMsg;          // msg to publish when a new CP starts;
    visualization_msgs::MarkerArray _predictions;
    visualization_msgs::MarkerArray _predictions_text;
    visualization_msgs::MarkerArray _predictions_velocity;
    visualization_msgs::MarkerArray _lastlap;
    visualization_msgs::MarkerArray _splineMarkersMatlab;
    std_msgs::Float32 _throttleViz;
    std_msgs::Float32 _steeringAngleViz;
    common_msgs::MPCInfo _mpcInfo;
    common_msgs::ModelInfoCompare _modelInfo;
    common_msgs::ModelInfo _realModel;
    common_msgs::ModelInfo _blendedModel;
    common_msgs::ModelInfo _diffModel;
    common_msgs::Output _outputDebug;
    common_msgs::OutputZI _outputZI;
    common_msgs::OutputZ _outputZ;
    common_msgs::OutputZN _outputZN;
    common_msgs::Horizon _horizon;
    common_msgs::TireModel _tireModel;
    common_msgs::NormalForces _normalForces;
    common_msgs::FrictionEllipse _frictionEllipseFront;
    common_msgs::FrictionEllipse _frictionEllipseRear;
    std_msgs::Float64 _diffGain;
    std::string _model_name;
    std::string _mission_config;
    int _simul;

    //misc variables
    visualization_msgs::Marker _marker_prediction;
    visualization_msgs::Marker _marker_prediction_text;
    visualization_msgs::Marker _marker_velocity;
    visualization_msgs::Marker _marker_lastlap;
    visualization_msgs::Marker _marker_spline;
    unsigned int _fail_counter = 10000;

    // NN variables
    common_msgs::NNData _nnData;

    // model learning variables
    float _Psi = 0.0f;
    Eigen::VectorXf _X_blended = Eigen::VectorXf::Zero(6);
    Eigen::VectorXf _X_real = Eigen::VectorXf::Zero(6);
    Eigen::VectorXf _X_diff = Eigen::VectorXf::Zero(6);
    Eigen::VectorXf _z_old = Eigen::VectorXf::Zero(4);
    Eigen::MatrixXf _safe_ctrl_actions = Eigen::MatrixXf::Zero(2, 31); // Change according to N

    // Check points variables
    unsigned int _checkpt_counter = 0;
    Eigen::VectorXf _checkpts;
    float _lap_landmark = 0.0f;

    // Cone hitting variables
    unsigned int _cone_hit_counter = 0;
    int _last_cone_hit_idx;
    Eigen::MatrixXf _cones;

    // SLAM auxiliar variables
    float _last_psi_clamped = 0.0f;
    int _winding_counter = 0;

    // data writing
    std::ofstream _data_file;
    bool _write_data_file;

    // for visualization of learning horizon
    int _mini_batch_size;

    ros::Time _start;
    std_msgs::Float64 _CPUtime;
};

std::vector<double> linspace(double start, double end, int numPoints);


#endif //LMPC_PIPELINE_HPP
