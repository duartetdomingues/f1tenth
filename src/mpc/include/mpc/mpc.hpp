#ifndef MPC_HPP
#define MPC_HPP

#include "rclcpp/rclcpp.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_mpc_model.h"
#include "blasfeo_d_aux_ext_dep.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#define N MPC_MODEL_N // Prediction horizon

class MPCNode : public rclcpp::Node
{
public:
    MPCNode();
    ~MPCNode();

    typedef struct
    {
        double x;
        double y;
        double yaw;
        double theta;
        double v;
    } State;

    typedef struct
    {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> yaw;
        std::vector<double> v;
    } ReferenceTrajectory;

private:
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void ProcessPose(const geometry_msgs::msg::Pose& msg);
    void VescServoCallback(const std_msgs::msg::Float64::SharedPtr msg);
    bool load_reference_trajectory_from_csv(const std::string &filename);
    void publish_reference_trajectory();
    void solveMPC();
    void set_trajectory_step();

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_vesc_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr solved_time_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr simulation_trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_pub_;

    std::array<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr, 5> state_vector_pub_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr, 2> control_vector_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomm_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vesc_servo_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    mpc_model_solver_capsule *capsule;

    double speed_to_erpm_gain;
    double speed_to_duty;
    double steering_angle_to_servo_gain;
    double steering_angle_to_servo_offset;

    double frequency;

    std::string frame_id_;

    bool use_pose_topic_;

    State current_state_;
    ReferenceTrajectory reference_trajectory_;

    std::array<std::reference_wrapper<std::vector<double>>, 4> reference_trajectory_vector_;

    // ACADOS variables
    ocp_nlp_config *nlp_config_;
    ocp_nlp_dims *nlp_dims_;
    ocp_nlp_in *nlp_in_;
    ocp_nlp_out *nlp_out_;
    ocp_nlp_solver *nlp_solver_;
    void *nlp_opts_;
};

#endif // MPC_HPP