#include "rclcpp/rclcpp.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // para tf2::getYaw()
#include <std_msgs/msg/float64.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h> // Add this include for tf2::Quaternion
#include <cmath>
#include <vector>

// Include ACADOS headers

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_mpc_model.h"

class MPCNode : public rclcpp::Node
{
public:
    MPCNode() : Node("mpc_node")
    {
        // Publishers and Subscribers
        control_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10);
        // state_sub_ = nh.subscribe("current_state", 10, &MPCNode::stateCallback, this);
        //  Update subscriptions to ROS 2 style

        odomm_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vesc/odom", 10, std::bind(&MPCNode::odomCallback, this, std::placeholders::_1));

        vesc_servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/vesc/servo_position_command", 10, std::bind(&MPCNode::VescServoCallback, this, std::placeholders::_1));
        // reference_sub_ = nh.subscribe("reference_trajectory", 10, &MPCNode::referenceCallback, this);
        create_reference_trajectory();

        // Initialize ACADOS solver
        nlp_config_ = mpc_model_acados_get_nlp_config(capsule);
        nlp_dims_ = mpc_model_acados_get_nlp_dims(capsule);
        nlp_in_ = mpc_model_acados_get_nlp_in(capsule);
        nlp_out_ = mpc_model_acados_get_nlp_out(capsule);
        nlp_solver_ = mpc_model_acados_get_nlp_solver(capsule);
        nlp_opts_ = mpc_model_acados_get_nlp_opts(capsule);

        if (mpc_model_acados_create(capsule) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create ACADOS solver.");
            rclcpp::shutdown();
        }

        // Set up a timer to solve MPC at a fixed frequency
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / frequency)),
            std::bind(&MPCNode::MPC_timerCallback, this));
    }

    ~MPCNode()
    {
        mpc_model_acados_free(capsule);
    }

    // Update the callback function signatures
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract state from odometry message
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        current_state_.yaw = q.getAngle(); // Extract yaw from quaternion
        current_state_.v = msg->twist.twist.linear.x;
    }

    void VescServoCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Extract servo position from VESC message
        current_state_.theta = msg->data;
    }

    /* void referenceCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        reference_trajectory_ = msg->data;
    } */

    void create_reference_trajectory()
    {
        // Create a simple reference trajectory for testing
#define N_total 100
        double n = (2 * M_PI) / N_total;
        double x_traj[N_total], y_traj[N_total], psi_traj[N_total];
        size_t index = 0;

        // Generate x and y trajectory
        for (double t = 0; t <= 2 * M_PI && index < N_total; t += n, ++index)
        {
            x_traj[index] = 2.5 * cos(t);
            y_traj[index] = 1.75 * sin(t);
        }

        // Calculate heading (psi) between consecutive points
        for (size_t i = 0; i < N_total - 1; ++i)
        {
            double dx = x_traj[i + 1] - x_traj[i];
            double dy = y_traj[i + 1] - y_traj[i];
            psi_traj[i] = atan2(dy, dx);
        }

        // Repeat the last heading value to match the size of x_traj and y_traj
        if (N_total > 1)
        {
            psi_traj[N_total - 1] = psi_traj[N_total - 2];
        }

        reference_trajectory_.x = std::vector<double>(x_traj, x_traj + N_total);
        reference_trajectory_.y = std::vector<double>(y_traj, y_traj + N_total);
        reference_trajectory_.yaw = std::vector<double>(psi_traj, psi_traj + N_total);
    }

    void MPC_timerCallback()
    {
        solveMPC();
    }

    void solveMPC()
    {
        #define N 10                 // Prediction horizon

        if (std::isnan(current_state_.x) || std::isnan(current_state_.y) || reference_trajectory_.x.empty() || reference_trajectory_.y.empty())
        {
            RCLCPP_WARN(this->get_logger(), "State or reference trajectory is empty. Skipping MPC solve.");
            return;
        }

        // 1. Find the closest point to the current position
        std::vector<double> distances(reference_trajectory_.x.size());
        for (size_t i = 0; i < reference_trajectory_.x.size(); ++i)
        {
            distances[i] = std::sqrt(std::pow(reference_trajectory_.x[i] - current_state_.x, 2) +
                                     std::pow(reference_trajectory_.y[i] - current_state_.y, 2));
        }
        auto idx_ref_start = std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));

        // 2. Compute cumulative distance along the trajectory
        std::vector<double> s_traj(reference_trajectory_.x.size(), 0.0);
        for (size_t i = 1; i < reference_trajectory_.x.size(); ++i)
        {
            double dx = reference_trajectory_.x[i] - reference_trajectory_.x[i - 1];
            double dy = reference_trajectory_.y[i] - reference_trajectory_.y[i - 1];
            s_traj[i] = s_traj[i - 1] + std::sqrt(dx * dx + dy * dy);
        }
        double traj_length = s_traj.back();

        // 3. Create target distances from the initial position
        double s0 = s_traj[idx_ref_start];
        std::vector<double> s_ref(N, 0.0);
        for (size_t i = 0; i < N; ++i)
        {
            s_ref[i] = i * 1/frequency* v_ref;
        }
        std::vector<double> s_target(N, 0.0);
        for (size_t i = 0; i < N; ++i)
        {
            s_target[i] = std::fmod(s0 + s_ref[i], traj_length);
        }

        // 4. Circular interpolation to find corresponding points
        std::vector<double> x_ref_step(N), y_ref_step(N), psi_ref(N);
        for (size_t i = 0; i < N; ++i)
        {
            x_ref_step[i] = linearInterpolation(s_traj, reference_trajectory_.x, s_target[i]);
            y_ref_step[i] = linearInterpolation(s_traj, reference_trajectory_.y, s_target[i]);
            psi_ref[i] = linearInterpolation(s_traj, reference_trajectory_.yaw, s_target[i]);
        }

        // 5. Set up the ACADOS solver
        for (size_t k = 0; k < N; ++k)
        {
            std::vector<double> yref_k = {
                x_ref_step[k],
                y_ref_step[k],
                std::sin(psi_ref[k]),
                std::cos(psi_ref[k]),
                0.0,
                v_ref};
            ocp_nlp_set(nlp_solver_, k, "yref", yref_k.data()); 
        }

        // Set terminal reference
        std::vector<double> yref_e = {
            x_ref_step.back(),
            y_ref_step.back(),
            std::sin(psi_ref.back()),
            std::cos(psi_ref.back())};
        ocp_nlp_set(nlp_solver_, N, "yref_e", yref_e.data());

        // Set initial state constraints
        ocp_nlp_set(nlp_solver_, 0, "lbx", current_state_vector_.data());
        ocp_nlp_set(nlp_solver_, 0, "ubx", current_state_vector_.data());

        // Solve the MPC problem
        int status = mpc_model_acados_solve(capsule);
        if (status != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "ACADOS solver failed with status %d", status);
            return;
        }

        // Get control output
        std::array<double, 2> control_output;
        ocp_nlp_get(nlp_solver_, "u", control_output.data()); // Retrieve control output
        
        ackermann_msgs::msg::AckermannDriveStamped control_msg;
        control_msg.header.stamp = this->get_clock()->now();
        control_msg.header.frame_id = "base_link";
        control_msg.drive.steering_angle = control_output[0] * steering_angle_to_servo_gain;
        control_msg.drive.acceleration = control_output[1] * speed_to_duty;

        // Publish control output
        control_pub_->publish(control_msg);
    }

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vesc_servo_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    mpc_model_solver_capsule *capsule = mpc_model_acados_create_capsule();


    double speed_to_erpm_gain = 4277.5;
    double speed_to_duty = 0.0602; //(m/s) / (duty cycle)
                                   // servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset
    double steering_angle_to_servo_gain = -0.840;  // -0.6984, -1.2135
    double steering_angle_to_servo_offset = 0.475; // right turn is positive

    double frequency = 10.0; // Frequency in Hz
    double v_ref = 1;        // Reference speed in m/s

    struct State
    {
        double x;
        double y;
        double yaw;
        double v;
        double theta;
    } current_state_;

    std::array<std::reference_wrapper<double>, 5> current_state_vector_ = {
        current_state_.x,
        current_state_.y,
        current_state_.yaw,
        current_state_.v,
        current_state_.theta};

    struct ReferenceTrajectory
    {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> yaw;
    } reference_trajectory_;

    std::array<std::reference_wrapper<std::vector<double>>, 3> reference_trajectory_vector_ = {
        reference_trajectory_.x,
        reference_trajectory_.y,
        reference_trajectory_.yaw};

    // ACADOS variables
    ocp_nlp_config *nlp_config_;
    ocp_nlp_dims *nlp_dims_;
    ocp_nlp_in *nlp_in_;
    ocp_nlp_out *nlp_out_;
    ocp_nlp_solver *nlp_solver_;
    void *nlp_opts_;

    double linearInterpolation(const std::vector<double> &x, const std::vector<double> &y, double target_x)
    {
        // Ensure the input vectors are valid
        if (x.size() != y.size() || x.empty())
        {
            throw std::invalid_argument("Input vectors must have the same size and cannot be empty.");
        }

        // Handle edge cases where target_x is outside the range of x
        if (target_x <= x.front())
        {
            return y.front();
        }
        if (target_x >= x.back())
        {
            return y.back();
        }

        // Find the interval [x[i], x[i+1]] that contains target_x
        for (size_t i = 0; i < x.size() - 1; ++i)
        {
            if (target_x >= x[i] && target_x <= x[i + 1])
            {
                // Perform linear interpolation
                double t = (target_x - x[i]) / (x[i + 1] - x[i]);
                return y[i] + t * (y[i + 1] - y[i]);
            }
        }

        // If no interval is found (should not happen), throw an exception
        throw std::runtime_error("Target value is out of interpolation range.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}