#include "../include/lmpc_handle.hpp"
// #include <std_msgs/Bool.h>
#include <iostream>

// bool start = false;

// void initializationCallback(const std_msgs::Bool &msg)
// {
//     start = msg.data;
//     if (start)
//     {
//         ROS_INFO("STARTING LEARNING MPCC!");
//     }
//     else
//     {
//         ROS_INFO("Waiting to start learning MPCC...");
//     }
// }

// void waitForInitialization(ros::NodeHandle &nh)
// {
//     std::string trigger_topic;
//     if (!ros::param::get("learning_mpcc/trigger_topic", trigger_topic))
//     {
//         ROS_WARN("Learning MPCC: trigger_topic not defined! Subscribing to /common/mission_tracker/path_planner_active");
//         trigger_topic = "/common/mission_tracker/path_planner_active";
//     }

//     ros::Subscriber sub_path_planner = nh.subscribe(trigger_topic, 1, &initializationCallback);

//     while (!start && ros::ok())
//     {
//         ros::spinOnce();
//     }

//     sub_path_planner.shutdown();
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "learning_mpcc");
    ros::NodeHandle nh;

    LearningMpccHandle learningMpccHandle(nh);

    double controller_frequency;

    // waitForInitialization(nh);

    if (!ros::param::get("learning_mpcc/controller_freq", controller_frequency)){
            ROS_ERROR("A failure occured during parameters loading");
            exit(EXIT_FAILURE);
    }

    // call service to set controller rate
    ros::ServiceClient controllerService = nh.serviceClient<common_msgs::controllerRate>("/control/controller/control_rate_service");
    // call mission tracker service that will set the controller frequency
    common_msgs::controllerRate controllerServiceMsg;
    controllerServiceMsg.request.controllerFreq = controller_frequency;
    if (controllerService.call(controllerServiceMsg)) {
        ROS_WARN("STA_Period has been updated with the current controller freq");
    }
    else {
        ROS_ERROR("STA_Period tracker controller rate service has failed");
    }

    ros::Rate loop_rate(controller_frequency); // Hz
    while (ros::ok())
    {
        ros::spinOnce();          // Runs callbacks first
        learningMpccHandle.run(); // Run L-MPC algorithm
        loop_rate.sleep();        // Sleeps until new cycle
    }

    learningMpccHandle.closeDataFile();

    return 0;
}