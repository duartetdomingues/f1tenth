#ifndef LMPC_HANDLE_HPP
#define LMPC_HANDLE_HPP

#include <ros/ros.h>
#include <lmpc_pipeline.hpp>

class LearningMpccHandle {

public:
    // Constructor
    LearningMpccHandle(ros::NodeHandle &nodeHandle);

    //Methods
    void advertiseToTopics();
    void subscribeToTopics();
    void publishToVisTopics();
    void run();
    void closeDataFile();

private:
    ros::NodeHandle _nodeHandle;

    //Publishers
    ros::Publisher _pubControlCmd;
    ros::Publisher _pubLearningData;
    ros::Publisher _pubNewLap;
    ros::Publisher _pubPredictions;
    ros::Publisher _pubPosition;
    ros::Publisher _pubPredictionsText;
    ros::Publisher _pubVisThrottle;
    ros::Publisher _pubVisSteeringAngle;
    ros::Publisher _pubHeart;
    ros::Publisher _pubMPCInfo;
    ros::Publisher _pubPredictionsVelocities;
    ros::Publisher _pubMPCModel;
    ros::Publisher _pubOutputMPC; 
    ros::Publisher _pubTire;
    ros::Publisher _pubNormalForces;
    ros::Publisher _pubFrictionEllipseFront;
    ros::Publisher _pubFrictionEllipseRear;
    ros::Publisher _pubNNData;
    ros::Publisher _pubDiffGain;
    ros::Publisher _pubPointToFollowMarker;
    ros::Publisher _pubPointAheadMarker;
    ros::Publisher _pubSplinePathMatlab;

    //Subscribers
    ros::Subscriber _subCarVelocity;
    ros::Subscriber _subSlamPose;
    ros::Subscriber _subLearningParams;
    ros::Subscriber _subMpcParams;
    ros::Subscriber _subControlMonitoring;
    ros::Subscriber _subInspectionSteering;
    ros::Subscriber _subSlamPath;
    ros::Subscriber _subLoopClosure;
    ros::Subscriber _subCarAcceleration;
    ros::Subscriber _subNNParams;
    ros::Subscriber _subGss;
    ros::Subscriber _subDashSteering;

    // for printing debug info
    bool _debug_info;

    LearningMpcc _learningMpcc;

    // Callbacks
    void carVelocityCallback(const common_msgs::CarVelocity &vel);
    void carAccelerationCallback(const sensor_msgs::Imu &acc);
    void slamPathCallback(const nav_msgs::Path &slamPath);
	void slamPoseCallback(const nav_msgs::Odometry &slamPose);
    void loopClosureCallback(const std_msgs::Bool &loopClosure);
    void learningParamsCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void mpcParamsCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void controlMonitoringCallback(const common_msgs::ControlMonitoring &msg);
    void inspectionSteeringCallback(const std_msgs::Float64 &inspectionSteering);
    void nnParamsCallback(const std_msgs::Float32MultiArray &nnParams);
    void dynamicReconfigureCallback(learning_mpcc::MPCParamsConfig &config, uint32_t level);
    void gssCallback(const common_msgs::GSS &gss);
    void dashCallback(const common_msgs::ControlCmd &dash);

    // Dynamic Parameters
    dynamic_reconfigure::Server<learning_mpcc::MPCParamsConfig> _server;
    dynamic_reconfigure::Server<learning_mpcc::MPCParamsConfig>::CallbackType _f;
};

#endif