#include "../include/lmpc_pure_pursuit.hpp"

PurePursuit::PurePursuit(): _pid(0.01, 1, -1, 0.15, 0, 0.01, 1)
{
    std::cout << "pure pursuit initialized" << std::endl;
	LoadParameters();

	_pid.setPIDParams((1/_controllerfrequency), _accelK,_accelKi, 0, _accelKlim);
	
}


void PurePursuit::LoadParameters(){

	if (!ros::param::get("learning_mpcc/controller_freq", _controllerfrequency)) {
		ROS_WARN_STREAM("Could not load controller/controller_freq.");
		_controllerfrequency = 20;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/longitudinal/accelerating_kp", _accelK)) {
		ROS_WARN_STREAM("Could not load controller/accelerating_kp. ");
		_accelK = 0.2;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/longitudinal/accelerating_ki", _accelKi)) {
		ROS_WARN_STREAM("Could not load controller/accelerating_ki.");
		_accelKi = 0.05;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/longitudinal/accelerating_klim", _accelKlim)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/accelerating_klim.");
		_accelKlim = 1;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/longitudinal/braking_k", _brakeK))	{
		ROS_WARN_STREAM("Could not load learning_mpcc/braking_k.");
		_brakeK = 0.5;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/lateral/steering_k", _steeringK)) {
		ROS_WARN_STREAM("Could not load learning_mpcc//steering_k.");
		_steeringK = 1.4;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/lateral/look_ahead_time", _lookAheadTime)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/look_ahead_time.");
		_lookAheadTime = 0.9;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/adaptive/further_look_ahead_time", _furtherLookAheadTime)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/further_look_ahead_time.");
		_furtherLookAheadTime = 1.5;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/lateral/min_lookahead_distance", _minDistanceSetPoint)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/min_lookahead_distance.");
		_minDistanceSetPoint = 5;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/adaptive/speed_angle", _speedAngle)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/speed_angle.");
		_speedAngle = 25;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/adaptive/max_speed", _maxSpeed)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/max_speed. ");
		_maxSpeed = 5.0;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/adaptive/min_speed", _minSpeed)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/min_speed. ");
		_minSpeed = 3.0;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/lateral/min_steering", _minStaSteering)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/min_steering. ");
		_minStaSteering = -140000.0;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/lateral/max_steering", _maxStaSteering)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/max_steering. ");
		_maxStaSteering = 140000.0;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/lateral/max_steering_change", _maxSteeringChange)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/max_steering_change. ");
		_maxSteeringChange = 0.01;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit/lateral/steering_filter_k", _steeringFilterK)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/steering_filter_k. ");
		_steeringFilterK = 0.5;
	}
	if (!ros::param::get("common/mission_selected", _mission)) {
		ROS_WARN_STREAM("Could not load mission. Trackdrive will be assumed");
		_mission = TRACKDRIVE;
	}
	if (!ros::param::get("learning_mpcc/use_throttle_curve", _useThrottleCurve)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/use_throttle_curve.");
		_useThrottleCurve = false;
	}
	if (!ros::param::get("learning_mpcc/throttle_init", _throttleInit)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/throttle_init. ");
		_throttleInit = 1;
	}
	if (!ros::param::get("learning_mpcc/throttle_max_distance", _throttleMaxDistance)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/throttle_max_distance. ");
		_throttleMaxDistance = 0;
	}

}

void PurePursuit::loadTrackdriveParams() {

	if (!ros::param::get("learning_mpcc/pure_pursuit_t/longitudinal/accelerating_kp", _accelK)) {
		ROS_WARN_STREAM("Could not load controller/accelerating_kp. ");
		_accelK = 0.2;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/longitudinal/accelerating_ki", _accelKi)) {
		ROS_WARN_STREAM("Could not load controller/accelerating_ki.");
		_accelKi = 0.05;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/longitudinal/accelerating_klim", _accelKlim)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/accelerating_klim.");
		_accelKlim = 1;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/longitudinal/braking_k", _brakeK)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/braking_k.");
		_brakeK = 0.5;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/lateral/steering_k", _steeringK)) {
		ROS_WARN_STREAM("Could not load learning_mpcc//steering_k.");
		_steeringK = 1.4;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/lateral/look_ahead_time", _lookAheadTime)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/look_ahead_time.");
		_lookAheadTime = 0.9;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/adaptive/further_look_ahead_time", _furtherLookAheadTime)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/further_look_ahead_time.");
		_furtherLookAheadTime = 1.5;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/lateral/min_lookahead_distance", _minDistanceSetPoint)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/min_lookahead_distance.");
		_minDistanceSetPoint = 5;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/adaptive/speed_angle", _speedAngle)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/speed_angle.");
		_speedAngle = 25;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/adaptive/max_speed", _maxSpeed)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/max_speed. ");
		_maxSpeed = 5.0;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/adaptive/min_speed", _minSpeed)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/min_speed. ");
		_minSpeed = 3.0;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/lateral/min_steering", _minStaSteering)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/min_steering. ");
		_minStaSteering = -140000.0;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/lateral/max_steering", _maxStaSteering)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/max_steering. ");
		_maxStaSteering = 140000.0;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/lateral/max_steering_change", _maxSteeringChange)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/max_steering_change. ");
		_maxSteeringChange = 0.01;
	}
	if (!ros::param::get("learning_mpcc/pure_pursuit_t/lateral/steering_filter_k", _steeringFilterK)) {
		ROS_WARN_STREAM("Could not load learning_mpcc/steering_filter_k. ");
		_steeringFilterK = 0.5;
	}

	_pid.setPIDParams((1/_controllerfrequency), _accelK,_accelKi, 0, _accelKlim);

	ROS_WARN("Loaded trackdrive parameters in MPC pure_pursuit");
}

/*************************Setters**********************/
void PurePursuit::setPath(const nav_msgs::Path &path)
{
    _slamPath = path;
}

void PurePursuit::setVelocity(const common_msgs::CarVelocity &vel)
{
   _currentVelocity = vel;
}

void PurePursuit::setOdometry(const nav_msgs::Odometry &odom)
{
    _carPose = odom;
}

/*************************Getters**********************/

common_msgs::ControlCmd const &PurePursuit::getControlCmd() const { return _controlCmd; }
visualization_msgs::Marker const &PurePursuit::getMarkerPointToFollow() const { return _markerPointToFollow; }
visualization_msgs::Marker const &PurePursuit::getMarkerPointAhead() const { return _markerPointAhead; }

/*************************Methods**********************/


void PurePursuit::runAlgorithm(){
	getNearestIndex(); // looks good getting the nearest index
	computeLateral();
	computeLongitudinal();
}


void PurePursuit::getNearestIndex(){

	double distance, minDistance = INFINITY;
	
	// look for the trajectory point that is closest to our current position
	for (int x = 0; x < _slamPath.poses.size(); x++) {

		double dx = _carPose.pose.pose.position.x - _slamPath.poses[x].pose.position.x;
		double dy = _carPose.pose.pose.position.y - _slamPath.poses[x].pose.position.y;
		distance = hypot(dx, dy);

		// 3 is track width
		if (distance < minDistance && distance < 3) {
			_index = x;
			minDistance = distance;
		}
		
	}
}

void PurePursuit::computeLateral(){	
	
    // first point (actual point to follow)
	_distanceSetPoint = getDistance(_lookAheadTime);
	
	//print distancesetpoint
	// std::cout << "Distance Set Point: " << _distanceSetPoint << std::endl;
	
	//get the point to follow index in the path 
	int iNext = point2Follow();

	// std::cout << iNext << std::endl; // this also looks good

	geometry_msgs::PoseStamped nextPoint = _slamPath.poses[iNext];

	//print next point coordinates
	// std::cout << "Next Point: " << nextPoint.pose.position.x << " " << nextPoint.pose.position.y << std::endl;
	
    double eta = std::atan2(nextPoint.pose.position.y - _carPose.pose.pose.position.y, nextPoint.pose.position.x - _carPose.pose.pose.position.x);
    eta = eta - getYawFromQuaternion(_carPose);
    eta = clamp_angle_pi_pi(eta);

    const double length = std::hypot(nextPoint.pose.position.y - _carPose.pose.pose.position.y, nextPoint.pose.position.x - _carPose.pose.pose.position.x - 0.81);

    float steeringAngleRef = static_cast<float>(_steeringK * std::atan(2.0 * 1.54 / length * std::sin(eta)));

    //limiting steering values using STA limits 
	if(steeringAngleRef >= _maxStaSteering/360537.0) {
		steeringAngleRef = _maxStaSteering/360537.0; 
	}
	else if (steeringAngleRef <= _minStaSteering/360537.0) {
		steeringAngleRef = _minStaSteering/360537.0;
	}

    _controlCmd.steering_angle = steeringAngleRef;

	_markerPointToFollow.pose.position.x = nextPoint.pose.position.x;
	_markerPointToFollow.pose.position.y = nextPoint.pose.position.y;
	_markerPointToFollow.color.r = 1.0;
	_markerPointToFollow.color.a = 1.0;
	_markerPointToFollow.type = visualization_msgs::Marker::SPHERE;
	_markerPointToFollow.header.frame_id = "map";
	_markerPointToFollow.header.stamp = ros::Time::now();
	_markerPointToFollow.scale.x = 0.4;
	_markerPointToFollow.scale.y = 0.4;
	_markerPointToFollow.scale.z = 0.4;
	_markerPointToFollow.color.b = 1.0;

}

void PurePursuit::computeLongitudinal(){

    _distanceSetPoint = getDistance(_furtherLookAheadTime);

    //get the point's index in the path 
    int iNext = point2Follow();
	geometry_msgs::PoseStamped nextPoint = _slamPath.poses[iNext];

    double eta = std::atan2(nextPoint.pose.position.y - _carPose.pose.pose.position.y, nextPoint.pose.position.x - _carPose.pose.pose.position.x);
	eta = eta - getYawFromQuaternion(_carPose);
	eta = clamp_angle_pi_pi(eta);

    const double vel = std::hypot(_currentVelocity.velocity.x, _currentVelocity.velocity.y); 

	double vel_ref = _maxSpeed - (exp(fabs(eta)) - 1) * (_maxSpeed - _minSpeed);
	if (vel_ref < _minSpeed) vel_ref = _minSpeed;
	_controlCmd.throttle = _pid.calculate(vel_ref,vel);

	_markerPointAhead.pose.position.x = nextPoint.pose.position.x;
	_markerPointAhead.pose.position.y = nextPoint.pose.position.y;
	_markerPointAhead.color.g = 1.0;
	_markerPointAhead.color.a = 1.0;	
	_markerPointAhead.type = visualization_msgs::Marker::SPHERE;
	_markerPointAhead.header.frame_id = "map";
	_markerPointAhead.header.stamp = ros::Time::now();
	_markerPointAhead.scale.x = 0.4;
	_markerPointAhead.scale.y = 0.4;
	_markerPointAhead.scale.z = 0.4;
	_markerPointAhead.color.r = 1.0;

}

int PurePursuit::point2Follow(){

    int iNext;
	double minDistance = INFINITY, distance;

	/** look for the trajectory point that is @param distanceSetPoint meters aways from our current position */
	for (int i = _index, x = 0; x < _slamPath.poses.size(); i++, x++){	
		if (i == _slamPath.poses.size()) i = 0;
		
		double dx = _slamPath.poses[_index].pose.position.x - _slamPath.poses[i].pose.position.x;
		double dy = _slamPath.poses[_index].pose.position.y - _slamPath.poses[i].pose.position.y;
		distance = fabs(std::hypot(dx, dy) - _distanceSetPoint);

		if (distance < minDistance){
			minDistance = distance;
			iNext = i;
		}else if(distance > minDistance + 1){
			break;
		}
	}

	return iNext;
}

float PurePursuit::getDistance(double time){

    double currentSpeed = std::hypot(_currentVelocity.velocity.x, _currentVelocity.velocity.y);
	double distanceSetPoint; 
    distanceSetPoint = currentSpeed * time;

	// if the distance is lower than minimum or speed = 0, set it to minimum
	if (currentSpeed == 0 || distanceSetPoint < _minDistanceSetPoint) distanceSetPoint = _minDistanceSetPoint;

    return distanceSetPoint;
}

double PurePursuit::getYawFromQuaternion(nav_msgs::Odometry pose) {
	
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