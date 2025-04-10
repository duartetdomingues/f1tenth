#include "ros/ros.h"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // para tf2::getYaw()
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Odometry.h>
#include "std_msgs/Bool.h"
#include <vector>

// Include ACADOS headers
extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_your_model_name.h" // Replace with your model name
}

class MPCNode
{
public:
    MPCNode()
    {
        // Initialize ROS node
        ros::NodeHandle nh;

        // Publishers and Subscribers
        control_pub_ = nh.advertise<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10);
        //state_sub_ = nh.subscribe("current_state", 10, &MPCNode::stateCallback, this);
        odomm_sub_ = nh.subscribe("/odometry/filtered", 10, &MPCNode::odomCallback, this);
        vesc_servo_sub_ = nh.subscribe("/vesc/servo_position_command", 10, &MPCNode::VescServoCallback, this);
        //reference_sub_ = nh.subscribe("reference_trajectory", 10, &MPCNode::referenceCallback, this);

        // Initialize ACADOS solver
        nlp_config_ = your_model_name_acados_get_nlp_config();
        nlp_dims_ = your_model_name_acados_get_nlp_dims();
        nlp_in_ = your_model_name_acados_get_nlp_in();
        nlp_out_ = your_model_name_acados_get_nlp_out();
        nlp_solver_ = your_model_name_acados_get_nlp_solver();
        nlp_opts_ = your_model_name_acados_get_nlp_opts();

        if (your_model_name_acados_create() != 0)
        {
            ROS_ERROR("Failed to create ACADOS solver.");
            ros::shutdown();
        }

        // Set up a timer to solve MPC at a fixed frequency
        double frequency = 10.0; // Frequency in Hz
        timer_ = nh.createTimer(ros::Duration(1.0 / frequency), &MPCNode::MPC_timerCallback, this);
    }

    ~MPCNode()
    {
        your_model_name_acados_free();
    }

    void odomCallback(const std_msgs::Odometry::ConstPtr &msg)
    {
        // Extract state from odometry message
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;
        current_state_.yaw = tf2::getYaw(msg->pose.pose.orientation);
        current_state_.v = msg->twist.twist.linear.x;
    }

    void VescServoCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        // Extract servo position from VESC message
        current_state_.theta = msg->data;
    }
    

    void referenceCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        reference_trajectory_ = msg->data;
    }

    void MPC_timerCallback(const ros::TimerEvent &)
    {
        solveMPC();
    }

    void solveMPC()
    {
        if (current_state_.empty() || reference_trajectory_.empty())
        {
            ROS_WARN("State or reference trajectory is empty. Skipping MPC solve.");
            return;
        }

        // Set initial state
        for (size_t i = 0; i < current_state_.size(); ++i)
        {
            ocp_nlp_set(nlp_config_, nlp_dims_, nlp_in_, "x0", current_state_vector.data());
        }

        // Set reference trajectory
        for (size_t i = 0; i < reference_trajectory_.size(); ++i)
        {
            ocp_nlp_set(nlp_config_, nlp_dims_, nlp_in_, "yref", reference_trajectory_.data());
        }

        // Solve the MPC problem
        int status = acados_solve();
        if (status != 0)
        {
            ROS_ERROR("ACADOS solver failed with status %d", status);
            return;
        }

        // Get control output
        std_msgs::Float64MultiArray control_msg;
        control_msg.data.resize(2); // Adjust size based on your control variables
        ocp_nlp_get(nlp_config_, nlp_dims_, nlp_out_, "u0", control_msg.data.data());

        // Publish control output
        control_pub_.publish(control_msg);
    }

private:
    ros::Publisher control_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber reference_sub_;

    std::array<double, 5> current_state_vector = {
        current_state_.x,
        current_state_.y,
        current_state_.yaw,
        current_state_.v,
        current_state_.theta
    };

    std::vector<double> reference_trajectory_;

    // ACADOS variables
    ocp_nlp_config *nlp_config_;
    ocp_nlp_dims *nlp_dims_;
    ocp_nlp_in *nlp_in_;
    ocp_nlp_out *nlp_out_;
    ocp_nlp_solver *nlp_solver_;
    void *nlp_opts_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_node");
    MPCNode mpc_node;
    ros::spin();
    return 0;
}