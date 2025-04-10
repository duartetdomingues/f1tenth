#include"tqvec_pipeline.hpp"

class TorqueVectoringHandle{

    public:

        //Constructor
        TorqueVectoringHandle(ros::NodeHandle &nodeHandle);

        // Methods
        void subscribeToTopics();
        void advertiseToTopics();
        void publishToTopics();

        void run();


    private:
        ros::NodeHandle _nodeHandle;

        ros::Publisher _pubTorques;
        ros::Publisher _pubSlips;
        ros::Publisher _pubOutputPID_slips;
        ros::Publisher _pubOutputPID_speeds;
        ros::Publisher _pubErrorSpeeds;
        ros::Publisher _pubYawRateRef;
        ros::Publisher _pubTVMPCInfo;
        ros::Publisher _pubHeart;
        ros::Publisher _pubTorqueDiffs;

        ros::Subscriber _subControlCmd;
        ros::Subscriber _subEstimatedWheelSpeed;
        ros::Subscriber _subCurrentVelocity;
        ros::Subscriber _subSlamPose;
        ros::Subscriber _subDiffGain;
        ros::Subscriber _subCurrentAccel;
        ros::Subscriber _subControlMonitoring;

        TorqueVectoring _torqueVectoring;

        bool _canPub = false;

        void ControlCmdCallback(const common_msgs::ControlCmd &controlCmd);
        void currentVelocityCallback(const common_msgs::CarVelocity &vel);
        void WheelSpeedCallback(const common_msgs::CarMotor &wheelSpeed);
        void slamPoseCallback(const nav_msgs::Odometry &slamPose);
        void diffGainCallback(const std_msgs::Float64 &diffGain);
        void currentAccelCallback(const sensor_msgs::Imu &accel);
        void controlMonitoringCallback(const common_msgs::ControlMonitoring &msg);

};