#include "rclcpp/rclcpp.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // para tf2::getYaw()
#include "nav_msgs/msg/odometry.hpp"
#include <vector>

// Include ACADOS headers

#include "acados_c/ocp_nlp_interface.h"
#include "mpc_solver/acados_solver_mpc_model.h"


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
        odomm_sub_ = nh.subscribe("/vesc/odom", 10, &MPCNode::odomCallback, this);
        vesc_servo_sub_ = nh.subscribe("/vesc/servo_position_command", 10, &MPCNode::VescServoCallback, this);
        //reference_sub_ = nh.subscribe("reference_trajectory", 10, &MPCNode::referenceCallback, this);
        create_reference_trajectory();
        

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

    void create_reference_trajectory()
    {
        // Create a simple reference trajectory for testing
        reference_trajectory_.clear();
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
            s_ref[i] = i * Ts * v_ref;
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
            x_ref_step[i] = interp1(s_traj, reference_trajectory_.x, s_target[i]);
            y_ref_step[i] = interp1(s_traj, reference_trajectory_.y, s_target[i]);
            psi_ref[i] = interp1(s_traj, reference_trajectory_.yaw, s_target[i]);
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
            v_ref
            };
            ocp_nlp_set(nlp_config_, nlp_dims_, nlp_in_, "yref", yref_k.data(), k);
        }

        // Set terminal reference
        std::vector<double> yref_e = {
            x_ref_step.back(),
            y_ref_step.back(),
            std::sin(psi_ref.back()),
            std::cos(psi_ref.back())
        };
        ocp_nlp_set(nlp_config_, nlp_dims_, nlp_in_, "yref_e", yref_e.data());

        // Set initial state constraints
        ocp_nlp_set(nlp_config_, nlp_dims_, nlp_in_, "lbx", current_state_vector.data(), 0);
        ocp_nlp_set(nlp_config_, nlp_dims_, nlp_in_, "ubx", current_state_vector.data(), 0);

        // Solve the MPC problem
        int status = acados_solve();
        if (status != 0)
        {
            ROS_ERROR("ACADOS solver failed with status %d", status);
            return;
        }

        // Get control output
        std::array<double, 2> control_output;
        ocp_nlp_get(nlp_config_, nlp_dims_, nlp_out_, "u", control_output.data(), 0);

        std_msgs::AckermannDriveStamped control_msg;
        control_msg.header.stamp = ros::Time::now();
        control_msg.header.frame_id = "base_link";
        control_msg.drive.steering_angle = control_output[0]*steering_angle_to_servo_gain;
        control_msg.drive.acceleration = control_output[1]* speed_to_duty;

        // Publish control output
        control_pub_.publish(control_msg);
    }

private:
    ros::Publisher control_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber reference_sub_;

    double speed_to_erpm_gain  = 4277.5;
    double speed_to_duty = 0.0602 //(m/s) / (duty cycle)
    // servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset
    double steering_angle_to_servo_gain = -0.840; // -0.6984, -1.2135
    double steering_angle_to_servo_offset = 0.475; // right turn is positive

    std::array<double, 5> current_state_vector = {
        current_state_.x,
        current_state_.y,
        current_state_.yaw,
        current_state_.v,
        current_state_.theta
    };

    std::array<std::vector<double>, 3> reference_trajectory_vector = {
        reference_trajectory_.x,
        reference_trajectory_.y,
        reference_trajectory_.yaw
    };

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