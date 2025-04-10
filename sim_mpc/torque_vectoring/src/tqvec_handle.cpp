#include"tqvec_handle.hpp"

TorqueVectoringHandle::TorqueVectoringHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle), _torqueVectoring(nodeHandle) {

    ROS_INFO("Constructed TorqueVectoring Handle");

    advertiseToTopics();
    subscribeToTopics();

}


void TorqueVectoringHandle::advertiseToTopics(){

    _pubTorques = _nodeHandle.advertise<common_msgs::CarMotor>("/control/torque_vectoring/torques", 1);
    _pubSlips = _nodeHandle.advertise<common_msgs::CarMotorFloat64>("/control/torque_vectoring/slip_ratios", 1);
    _pubOutputPID_slips = _nodeHandle.advertise<common_msgs::CarMotor>("/control/torque_vectoring/output_PID_slips", 1);
    _pubOutputPID_speeds = _nodeHandle.advertise<common_msgs::CarMotor>("/control/torque_vectoring/output_PID_speeds", 1);
    _pubErrorSpeeds = _nodeHandle.advertise<std_msgs::Float64>("/control/torque_vectoring/wheel_speeds_error", 1);
    _pubYawRateRef = _nodeHandle.advertise<std_msgs::Float64>("/control/torque_vectoring/yaw_rate_ref", 1);
    _pubTVMPCInfo = _nodeHandle.advertise<common_msgs::MPCInfo>("/control/torque_vectoring/tvmpc_info", 1);
    _pubHeart = _nodeHandle.advertise<std_msgs::Float64>("/heartbeat/raw", 1);
    _pubTorqueDiffs = _nodeHandle.advertise<std_msgs::Float32MultiArray>("/control/torque_vectoring/torque_diffs", 1);
}

void TorqueVectoringHandle::subscribeToTopics(){

    _subControlCmd = _nodeHandle.subscribe("/control/controller/control_cmd", 1 , &TorqueVectoringHandle::ControlCmdCallback, this, ros::TransportHints().tcpNoDelay());
    // front and rear wheel speeds
    _subEstimatedWheelSpeed = _nodeHandle.subscribe("/estimation/wheel_speeds",1,&TorqueVectoringHandle::WheelSpeedCallback,this);
    _subCurrentVelocity = _nodeHandle.subscribe("/estimation/state_estimation/velocity", 1, &TorqueVectoringHandle::currentVelocityCallback, this);
    _subSlamPose = _nodeHandle.subscribe("/estimation/slam/odometry", 1, &TorqueVectoringHandle::slamPoseCallback, this);
    _subDiffGain = _nodeHandle.subscribe("/control/learning_mpcc/diff_gain", 1, &TorqueVectoringHandle::diffGainCallback, this);
    _subCurrentAccel = _nodeHandle.subscribe("/estimation/accel_gravity", 1, &TorqueVectoringHandle::currentAccelCallback, this);
    // monitoring topics
   _subControlMonitoring = _nodeHandle.subscribe("/common/mission_tracker/control_monitoring", 1, &TorqueVectoringHandle::controlMonitoringCallback, this);
}

void TorqueVectoringHandle::publishToTopics(){

    _pubSlips.publish(_torqueVectoring.getSlips());
    _pubOutputPID_slips.publish(_torqueVectoring.getOutputPID_slips());
    _pubOutputPID_speeds.publish(_torqueVectoring.getOutputPID_speeds());
    _pubErrorSpeeds.publish(_torqueVectoring.getSpeedError());
    _pubYawRateRef.publish(_torqueVectoring.getYawRateRef());
    _pubTVMPCInfo.publish(_torqueVectoring.getMPCInfo());
    _pubTorqueDiffs.publish(_torqueVectoring.getTorqueDiffs());
    _pubTorques.publish(_torqueVectoring.getTorques());
    _pubHeart.publish(_torqueVectoring.getCPUTime());
}

void TorqueVectoringHandle::ControlCmdCallback(const common_msgs::ControlCmd &controlCmd){
  _torqueVectoring.setControlCmd(controlCmd);
}


void TorqueVectoringHandle::WheelSpeedCallback(const common_msgs::CarMotor &wheelSpeed){
  _torqueVectoring.setWheelSpeed(wheelSpeed);
}

void TorqueVectoringHandle::currentVelocityCallback(const common_msgs::CarVelocity &vel){
  //ROS_INFO("Received ahrs velocity");
  _torqueVectoring.setCurrentVelocity(vel);
}

void TorqueVectoringHandle::slamPoseCallback(const nav_msgs::Odometry &slamPose){
  //ROS_INFO("Received slam pose");
  _torqueVectoring.setSlamPose(slamPose);
}

void TorqueVectoringHandle::currentAccelCallback(const sensor_msgs::Imu &accel){
  _torqueVectoring.setCurrentAccel(accel);
}

/**
*	Name: controlMonitoringCallback.
*	Description: Callback to update the cars control state.
*	Inputs: Control Monitoring message.
*	Output: void.
*/
void TorqueVectoringHandle::controlMonitoringCallback(const common_msgs::ControlMonitoring &msg) {
  //ROS_INFO("Received control monitoring");
  _torqueVectoring.changeControllerState(msg);
}

void TorqueVectoringHandle::diffGainCallback(const std_msgs::Float64 &diffGain){
  _torqueVectoring.setDiffGain(diffGain);
}

void TorqueVectoringHandle::run(){
  _torqueVectoring.run();
  publishToTopics();
}

