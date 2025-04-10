#include "ros/ros.h"
#include "tqvec_handle.hpp"
#include "common_msgs/inverterControlMode.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "torque_vectoring");
  ros::NodeHandle nh;

  // call service to set controller rate
  ros::ServiceClient inverterControlModeService = nh.serviceClient<common_msgs::inverterControlMode>("/control/controller/inverter_control_mode");
  // call mission tracker service that will set the controller frequency
  common_msgs::inverterControlMode inverterControlModeServiceMsg;
  inverterControlModeServiceMsg.request.inputMode = 0;
  if (inverterControlModeService.call(inverterControlModeServiceMsg)) {
      ROS_WARN("CMD sent to PDU");
  }
  else {
    ROS_ERROR("CMD failed to be sent to PDU");
  }

  TorqueVectoringHandle TorqueVectoringHandle(nh);

  ros::Rate loop_rate(100);
  while (ros::ok()) {

    TorqueVectoringHandle.run();

    ros::spinOnce();           
    loop_rate.sleep();         
  }
  return 0;
}

