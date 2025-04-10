#include <vector>

// Messages
#include "nav_msgs/Path.h"
#include "common_msgs/CarVelocity.h"
#include "nav_msgs/Odometry.h"
#include "common_msgs/ControlCmd.h"
#include "visualization_msgs/Marker.h"

#include <ros/ros.h>
#include <tf/tf.h>

#include <lmpc_pid.hpp>
#include "as_lib/common.h"


class PurePursuit
{
public:

    PurePursuit();
    //~PurePursuit();

    void LoadParameters();
    void loadTrackdriveParams();

    // setters
    void setPath(const nav_msgs::Path &path);
    void setVelocity(const common_msgs::CarVelocity &vel);
    void setOdometry(const nav_msgs::Odometry &odom);


    // getters 
    common_msgs::ControlCmd const & getControlCmd() const;
    visualization_msgs::Marker const & getMarkerPointToFollow() const;
    visualization_msgs::Marker const & getMarkerPointAhead() const;

    void runAlgorithm();



private:

    // atributtes
    nav_msgs::Path _slamPath;
    common_msgs::CarVelocity _currentVelocity;
    nav_msgs::Odometry _carPose;
    common_msgs::ControlCmd _controlCmd;

    // pid class
    PID _pid;

    //parameters
    double _minDistanceSetPoint;
    double _maxSteering;
    double _maxSteeringChange;
    double _steeringFilterK;
    double _throttleInit;
    double _throttleMaxDistance;
    bool _useThrottleCurve;
    int _mission;
    double _controllerfrequency;
    double _accelK;
    double _accelKi;
    double _accelKlim;
    double _brakeK;
    double _steeringK;
    double _lookAheadTime;
    double _furtherLookAheadTime;
    double _speedAngle;
    double _maxSpeed;
    double _minSpeed;
    double _minStaSteering;
    double _maxStaSteering;

    double _index;
    double _distanceSetPoint;
    
    // methods
    void getNearestIndex();
    void loadparameters();
    void computeLateral();
    void computeLongitudinal();
    int point2Follow();
    float getDistance(double);

    //viz 
    void visualize();
    visualization_msgs::Marker _markerPointToFollow;
    visualization_msgs::Marker _markerPointAhead;

    // auxiliar methods
    double getYawFromQuaternion(nav_msgs::Odometry);

    static inline float clamp_angle_pi_pi(float angle) {
        return static_cast<float>(angle - 2 * M_PI * floor((angle + M_PI) / (2 * M_PI)));
    }


    double test;

};