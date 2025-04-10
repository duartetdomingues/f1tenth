#include "../include/lmpc_handle.hpp"

LearningMpccHandle::LearningMpccHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle)
{
  ROS_INFO("Constructing learning_mpcc Handle");

  advertiseToTopics();
  subscribeToTopics();

  // get debug info parameter
  if (!ros::param::get("learning_mpcc/debug/learning_mpcc", _debug_info))
  {
    ROS_WARN("Learning MPCC: debug info not defined, not showing debug info by default.");
    _debug_info = false;
  }

  _f = boost::bind(&LearningMpccHandle::dynamicReconfigureCallback, this, _1, _2);
  _server.setCallback(_f);

  ROS_WARN("LMPC Handle constructor finished.");
}
/**
*	Name: advertiseToTopics.
*	Description: Creates all the necessary ROS publishers. It's also where the name of the topics is defined.
*	Input's: none
*	Output: void
*/
void LearningMpccHandle::advertiseToTopics()
{
  ROS_INFO("learning_mpcc will advertise to topics");
  _pubControlCmd = _nodeHandle.advertise<common_msgs::ControlCmd>("/control/controller/control_cmd", 2);
  _pubLearningData = _nodeHandle.advertise<std_msgs::Float32MultiArray>("/control/learning_mpcc/learning/learning_data", 1);
  _pubNewLap = _nodeHandle.advertise<std_msgs::UInt8MultiArray>("/control/learning_mpcc/learning/new_lap", 1);
  _pubPredictions = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/control/learning_mpcc/vis/prediction_points", 1);
  _pubPredictionsVelocities = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/control/learning_mpcc/vis/prediction_points_velocities", 1);
  _pubPredictionsText = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/control/learning_mpcc/vis/prediction_points_text", 1);
  _pubPosition = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/control/learning_mpcc/vis/last_lap", 1);
  _pubVisThrottle = _nodeHandle.advertise<std_msgs::Float32>("/control/controller/vis/throttle", 1);
  _pubVisSteeringAngle = _nodeHandle.advertise<std_msgs::Float32>("/control/controller/vis/steering_angle", 1);
  _pubHeart = _nodeHandle.advertise<std_msgs::Float64>("/heartbeat/raw", 1);
  _pubMPCInfo = _nodeHandle.advertise<common_msgs::MPCInfo>("/control/learning_mpcc/debug_info", 1);
  _pubMPCModel = _nodeHandle.advertise<common_msgs::ModelInfoCompare>("/control/learning_mpcc/mpc_model", 1);
  _pubOutputMPC = _nodeHandle.advertise<common_msgs::Output>("/control/learning_mpcc/mpc_output", 1);
  _pubTire = _nodeHandle.advertise<common_msgs::TireModel>("/control/learning_mpcc/tire_model", 1);
  _pubNormalForces = _nodeHandle.advertise<common_msgs::NormalForces>("/control/learning_mpcc/normal_forces", 1);
  _pubFrictionEllipseFront = _nodeHandle.advertise<common_msgs::FrictionEllipse>("/control/learning_mpcc/friction_ellipse_front", 1);
  _pubFrictionEllipseRear = _nodeHandle.advertise<common_msgs::FrictionEllipse>("/control/learning_mpcc/friction_ellipse_rear", 1);
  _pubNNData = _nodeHandle.advertise<common_msgs::NNData>("/control/learning_mpcc/nn_data", 1);
  _pubDiffGain = _nodeHandle.advertise<std_msgs::Float64>("/control/learning_mpcc/diff_gain", 1);
  _pubSplinePathMatlab = _nodeHandle.advertise<visualization_msgs::MarkerArray>("/control/learning_mpcc/spline_path_matlab", 1);

  _pubPointToFollowMarker = _nodeHandle.advertise<visualization_msgs::Marker>("/control/learning_mpcc/vis/point_to_follow", 1);
  _pubPointAheadMarker = _nodeHandle.advertise<visualization_msgs::Marker>("/control/learning_mpcc/vis/point_ahead", 1);

}
/**
*	Name: subscribeToTopics.
*	Description: Subscribes to the car velocity and odometry topics. If in simulation, the ground truth might be used.
*	Input's: none
*	Output: void
*/
void LearningMpccHandle::subscribeToTopics()
{

  // Car velocity topic
  _subCarVelocity = _nodeHandle.subscribe("/estimation/state_estimation/velocity", 1, &LearningMpccHandle::carVelocityCallback, this);

  // Car acceleration topic
  _subCarAcceleration = _nodeHandle.subscribe("/estimation/accel_gravity", 1, &LearningMpccHandle::carAccelerationCallback, this);

  // Subscribes to the correspondent odometry topic
  _subSlamPose = _nodeHandle.subscribe("/estimation/slam/odometry", 1, &LearningMpccHandle::slamPoseCallback, this);

  // Centerline
  _subSlamPath = _nodeHandle.subscribe("/estimation/slam/centerline", 1, &LearningMpccHandle::slamPathCallback, this, ros::TransportHints().tcpNoDelay());

  _subLoopClosure = _nodeHandle.subscribe("/estimation/slam/loop_closure", 1, &LearningMpccHandle::loopClosureCallback, this);
  // // Learning parameters topic
  // _subLearningParams = _nodeHandle.subscribe("/control/learning_mpcc/learning/learning_params", 1, &LearningMpccHandle::learningParamsCallback, this);

  // // MPC parameters topic
  // _subMpcParams = _nodeHandle.subscribe("/control/learning_mpcc/learning/mpc_params", 1, &LearningMpccHandle::mpcParamsCallback, this);

  // monitoring topics
  _subControlMonitoring = _nodeHandle.subscribe("/common/mission_tracker/control_monitoring", 1, &LearningMpccHandle::controlMonitoringCallback, this);

  // inspection
  _subInspectionSteering = _nodeHandle.subscribe("/common/mission_tracker/inspection_steering", 1, &LearningMpccHandle::inspectionSteeringCallback, this);

  // nn parameters
  _subNNParams = _nodeHandle.subscribe("/control/model_learning/nn_params", 1, &LearningMpccHandle::nnParamsCallback, this);

  _subGss = _nodeHandle.subscribe("/common/can_sniffer/gss", 1, &LearningMpccHandle::gssCallback, this);

  _subDashSteering = _nodeHandle.subscribe("/control/controller/steering_actual", 1, &LearningMpccHandle::dashCallback, this); 
}
/**
*	Name: run
*	Description: runs the main algorithm
*	Inputs: none
*	Output: void
*/

void LearningMpccHandle::gssCallback(const common_msgs::GSS &gss){
  common_msgs::CarVelocity vel_msg;

  vel_msg.velocity.x = gss.velocity.x;
  vel_msg.velocity.y = gss.corrected_velocity.y;
  vel_msg.velocity.theta = gss.corrected_velocity.theta;

  // ROS_WARN("Receiving VELOCITY");
  // _learningMpcc.setCurrentVelocity(vel_msg);

}

void LearningMpccHandle::run()
{
  // ROS_WARN("learning_mpcc HANDLE is running!!!")

  if(_learningMpcc._publish){
    _pubControlCmd.publish(_learningMpcc.getControlCommand()); // urgent topic
    _pubNNData.publish(_learningMpcc.getNNData());
  }

  _learningMpcc.run();

  _pubPointToFollowMarker.publish(_learningMpcc.getMarkerPointToFollow());
  _pubPointAheadMarker.publish(_learningMpcc.getMarkerPointAhead());

  _learningMpcc.setInitialGuess();

  _learningMpcc.setMarkerArray();

  _learningMpcc.writeDataToFile();

  _pubDiffGain.publish(_learningMpcc.getDiffGain());

  publishToVisTopics();

  if (_debug_info)
    _learningMpcc.lmpcInfoDebug();

  _pubMPCModel.publish(_learningMpcc.getMPCModel());
  _pubOutputMPC.publish(_learningMpcc.getOutputMPC());
  _pubTire.publish(_learningMpcc.getTire());
  _pubNormalForces.publish(_learningMpcc.getNormalForces());
  _pubFrictionEllipseFront.publish(_learningMpcc.getFrictionEllipseFront());
  _pubFrictionEllipseRear.publish(_learningMpcc.getFrictionEllipseRear());

}
/**
*	Name: publishToVisTopics
*	Description: publishes visualization related info.
*	Inputs: none
*	Output: void
*/
void LearningMpccHandle::publishToVisTopics()
{
  // _pubPredictions.publish(_learningMpcc.getPredictions());
  _pubPredictionsVelocities.publish(_learningMpcc.getPredictionsVelocities());
  // _pubPosition.publish(_learningMpcc.getPosition());
  // _pubPredictionsText.publish(_learningMpcc.getPredictionsText());
  _pubVisThrottle.publish(_learningMpcc.getThrottle());
  _pubVisSteeringAngle.publish(_learningMpcc.getSteeringAngle());
  _pubHeart.publish(_learningMpcc.getCPUtime());
  _pubMPCInfo.publish(_learningMpcc.getMPCInfo());
  _learningMpcc.reset_trained_flag();
  _pubSplinePathMatlab.publish(_learningMpcc.getSplineMarkersMatlab());
}

/**
*	Name: carVelocityCallback.
*	Description: Callback to update the car's pose.
*	Inputs: Car velocity message of the type common_msgs::CarVelocity
*	Output: void
*/
void LearningMpccHandle::carVelocityCallback(const common_msgs::CarVelocity &vel)
{
  // ROS_WARN("Receiving VELOCITY");
  _learningMpcc.setCurrentVelocity(vel);
}

void LearningMpccHandle::dashCallback(const common_msgs::ControlCmd &dash){

  _learningMpcc.setDashSteering(dash);

}

void LearningMpccHandle::carAccelerationCallback(const sensor_msgs::Imu &accel)
{
  // ROS_WARN("Receiving ACCELERATION");
  _learningMpcc.setCurrentAcceleration(accel);
}

/**
*	Name: slamPoseCallback.
*	Description: Callback to update the car's pose.
*	Inputs: Odometry message of the type nav_msgs::Odometry
*	Output: void
*/
void LearningMpccHandle::slamPoseCallback(const nav_msgs::Odometry &slamPose)
{
  // ROS_WARN("Receiving POSE");
  _learningMpcc.setSlamPose(slamPose);
}

void LearningMpccHandle::slamPathCallback(const nav_msgs::Path &slamPath)
{
  _learningMpcc.setSlamPath(slamPath);
}

void LearningMpccHandle::loopClosureCallback(const std_msgs::Bool &loopClosure) {
  _learningMpcc.setLoopClosure(loopClosure);
}

// /**
// *	Name: learningParamsCallback
// *	Description: Callback to update model learning parameters
// *	Inputs: parameters message
// *	Output: void
// */
// void LearningMpccHandle::learningParamsCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
// {
//   _learningMpcc.setMLparams(msg);
// }
// /**
// *	Name: mpcParamsCallback
// *	Description: Callback to MPC parameters
// *	Inputs: MPC parameters message
// *	Output: void
// */
// void LearningMpccHandle::mpcParamsCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
// {
//   _learningMpcc.setMpcParams(msg);
// }

/**
*	Name: controlMonitoringCallback.
*	Description: Callback to update the controller's state.
*	Input's: Control state of the type common_msgs::ControlMonitoring
*	Output: void
*/
void LearningMpccHandle::controlMonitoringCallback(const common_msgs::ControlMonitoring &msg) {
  _learningMpcc.changeLmpcState(msg);
}

/**
*	Name: closeDataFile
*	Description: closes data file
*	Inputs: none
*	Output: void
*/
void LearningMpccHandle::closeDataFile()
{
  _learningMpcc.closeDataFile();
}

/**
*	Name: inspectionSteeringCallback.
*	Description: Callback to set the steering references for the inspection mission.
*	Inputs: Steering of type std_msgs::Float64.
*	Output: void.
*/
void LearningMpccHandle::inspectionSteeringCallback(const std_msgs::Float64 &inspectionSteering) {
  //ROS_INFO("Received inspection steering");
  _learningMpcc.setInspectionSteering(inspectionSteering);
}

void LearningMpccHandle::nnParamsCallback(const std_msgs::Float32MultiArray &nnParams) {
  //ROS_INFO("Received NN params");
  _learningMpcc.setNNparams(nnParams);
}

void LearningMpccHandle::dynamicReconfigureCallback(learning_mpcc::MPCParamsConfig &config, uint32_t level){
  if(!_learningMpcc._initParams){
    _learningMpcc.setMPCparams(config);
  }
  _learningMpcc._initParams = false;
}
