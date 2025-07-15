#include "mpc/mpc.hpp"

MPCNode::MPCNode() : Node("mpc_node"),
                     capsule(mpc_model_acados_create_capsule()),
                     speed_to_erpm_gain(4277.5),
                     speed_to_duty(0.0602),
                     steering_angle_to_servo_gain(-0.840),
                     steering_angle_to_servo_offset(0.475),
                     frequency(10.0),
                     reference_trajectory_vector_({reference_trajectory_.x, reference_trajectory_.y, reference_trajectory_.yaw, reference_trajectory_.v}),
                     tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
                     tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{

    // Declare parameters
    this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    this->declare_parameter<std::string>("traj_file", "/home/jetson/f1tenth/src/traj/pts/oito_n1000_a2.5_b1.8_v1.0.csv");
    this->declare_parameter<std::string>("frame_id", "odom");
    this->declare_parameter<std::string>("pose_topic", "/pose");

    // Retrieve parameters
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string traj_file = this->get_parameter("traj_file").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    std::string pose_topic_ = this->get_parameter("pose_topic").as_string();
    if (pose_topic_ == "")
    {
        use_pose_topic_ = false;
    }
    else
    {
        use_pose_topic_ = true;
    }

    // Publishers and Subscribers
    control_vesc_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10);
    solved_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("mpc/solved_time", 10);
    ref_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc/ref_path", 10);
    reference_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc/ref_traj", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/state_array", 10);
    control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/control_array", 10);
    control_vector_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/control_vector", 10);
    state_vector_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/state_vector", 10);

    odomm_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&MPCNode::OdomCallback, this, std::placeholders::_1));

    if (use_pose_topic_)
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 10, std::bind(&MPCNode::PoseCallback, this, std::placeholders::_1));
    }
    vesc_servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/vesc/servo_position_command", 10, std::bind(&MPCNode::VescServoCallback, this, std::placeholders::_1));

    // Load reference trajectory
    if (!load_reference_trajectory_from_csv(traj_file))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load reference trajectory.");
        rclcpp::shutdown();
    }
    publish_reference_trajectory();

    // Create ACADOS solver capsule
    if (mpc_model_acados_create(capsule) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create ACADOS solver.");
        rclcpp::shutdown();
    }

    // Initialize ACADOS solver
      nlp_config_ = mpc_model_acados_get_nlp_config(capsule);
    nlp_dims_ = mpc_model_acados_get_nlp_dims(capsule);
    nlp_in_ = mpc_model_acados_get_nlp_in(capsule);
    nlp_out_ = mpc_model_acados_get_nlp_out(capsule);
    nlp_solver_ = mpc_model_acados_get_nlp_solver(capsule);
    nlp_opts_ = mpc_model_acados_get_nlp_opts(capsule); 

    // Set weights for the cost function
    // std::vector<double> cost_weights = {1.0, 1.0, 0.0, 0.0, 0.0, 0.0}; // Example weights for x, y, sin(psi), cos(psi), steering, velocity
    /* std::vector<double> cost_weights = {10.0, 10.0, 1.0, 1.0, 1.0, 1.0}; // Example weights for x, y, sin(psi), cos(psi), steering, velocity
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(6, 6);
    W.diagonal() << cost_weights[0], cost_weights[1], cost_weights[2], cost_weights[3], cost_weights[4], cost_weights[5];
    for (size_t k = 0; k < N; k++)
    {
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, k, "W", W.data());
    } */
    // Set final cost weights
    // std::vector<double> final_cost_weights = {1.0, 1.0, 0.0, 0.0}; // Example weights for x, y, sin(psi), cos(psi), steering, velocity
    /* std::vector<double> final_cost_weights = {10.0, 10.0, 1.0, 1.0}; // Example weights for x, y, sin(psi), cos(psi), steering, velocity
    Eigen::MatrixXd Wf = Eigen::MatrixXd::Zero(4, 4);
    Wf.diagonal() << final_cost_weights[0], final_cost_weights[1], final_cost_weights[2], final_cost_weights[3];
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N, "W", Wf.data()); */

    // nlp_config_ = ocp_nlp_config_create(nlp_dims_);
    // nlp_dims_ = ocp_nlp_dims_create(nlp_config_);
    // nlp_in_ = ocp_nlp_in_create(nlp_config_, nlp_dims_);
    // nlp_out_ = ocp_nlp_out_create(nlp_config_, nlp_dims_);
    // nlp_opts_ = ocp_nlp_solver_opts_create(nlp_config_, nlp_dims_);
    // nlp_solver_ = ocp_nlp_solver_create(nlp_config_, nlp_dims_, nlp_opts_, nlp_in_);

    // precompute
    // ocp_nlp_precompute(nlp_solver_, nlp_in_, nlp_out_);

    // Set up a timer to solve MPC at a fixed frequency
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / frequency)),
        std::bind(&MPCNode::solveMPC, this));
}

MPCNode::~MPCNode()
{
    mpc_model_acados_free(capsule);
}

void MPCNode::solveMPC()
{
    if (N <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid number of stages in MPC model.");
        return;
    }

    if (std::isnan(current_state_.x) || std::isnan(current_state_.y) || reference_trajectory_.x.empty() || reference_trajectory_.y.empty())
    {
        RCLCPP_WARN(this->get_logger(), "State or reference trajectory is empty. Skipping MPC solve.");
        return;
    }

    // Get the reference trajectory to horizont
    set_trajectory_step();

    // Set initial state constraints
    std::vector<double> current_state_vector_ = {
        current_state_.x,
        current_state_.y,
        current_state_.yaw,
        current_state_.theta,
        current_state_.v};

    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", current_state_vector_.data());
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", current_state_vector_.data());

     // Publish state array
    std_msgs::msg::Float64MultiArray state_array_msg;
    state_array_msg.data = {current_state_vector_[0],
                            current_state_vector_[1],
                            current_state_vector_[2],
                            current_state_vector_[3],
                            current_state_vector_[4]};
    state_pub_->publish(state_array_msg);

    // Solve the MPC problem
    int status = ocp_nlp_solve(nlp_solver_, nlp_in_, nlp_out_);
    //int status = mpc_model_acados_solve(capsule);
    // mpc_model_acados_print_stats(capsule);
    /* std::vector<double> xtraj((N + 1) * 6);
    std::vector<double> utraj(N * 2);
    for (int ii = 0; ii <= N; ii++)
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj[ii*6]);
    for (int ii = 0; ii < N; ii++)
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj[ii*2]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( 6, N+1, xtraj.data(), 6);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( 2, N, utraj.data(), 2 ); */
    if (status != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "ACADOS solver failed with status %d", status);
        return;
    }

    // Get control output
    std::array<double, 2> control_output;
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", control_output.data()); // Retrieve control output

    double solved_time;
    ocp_nlp_get(nlp_solver_, "time_tot", &solved_time);

    std_msgs::msg::Float64 solved_time_msg;
    solved_time_msg.data = solved_time;
    solved_time_pub_->publish(solved_time_msg);

    ackermann_msgs::msg::AckermannDriveStamped control_vesc_msg;
    control_vesc_msg.header.stamp = this->get_clock()->now();
    control_vesc_msg.header.frame_id = "base_link";
    control_vesc_msg.drive.steering_angle = control_output[0] * steering_angle_to_servo_gain;
    control_vesc_msg.drive.acceleration = control_output[1] * speed_to_duty;

    // Publish control output
    control_vesc_pub_->publish(control_vesc_msg);

    // Publish control array
    std_msgs::msg::Float64MultiArray control_array_msg;
    control_array_msg.data = {control_output[0], control_output[1]};
    control_pub_->publish(control_array_msg);

    // Publish control vector
    std::vector<double> control_vector(N * 2);
    for (int ii = 0; ii < N; ii++)
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &control_vector[ii * 2]);
    std_msgs::msg::Float64MultiArray control_vector_msg;
    control_vector_msg.data = control_vector;
    control_vector_pub_->publish(control_vector_msg);

    // Publish state vector
    std::vector<double> state_vector((N + 1) * 6);
    for (int ii = 0; ii <= N; ii++)
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &state_vector[ii * 6]);
    std_msgs::msg::Float64MultiArray state_vector_msg;
    state_vector_msg.data = state_vector;
    state_vector_pub_->publish(state_vector_msg);
} // RCLCPP_INFO(this->get_logger(), "Current steering angle: %f", current_state_.steering_angle);

void MPCNode::set_trajectory_step()
{
    // 1. Find the closest point to the current position
    std::vector<double> distances(reference_trajectory_.x.size());
    for (size_t i = 0; i < reference_trajectory_.x.size(); ++i)
    {
        distances[i] = std::sqrt(std::pow(reference_trajectory_.x[i] - current_state_.x, 2) +
                                 std::pow(reference_trajectory_.y[i] - current_state_.y, 2));
    }
    
    auto idx_ref_start = std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));

    // 2. Compute reference points by walking along the trajectory
    std::vector<double> x_ref_step(N), y_ref_step(N), psi_ref(N), v_ref(N);
    size_t idx = idx_ref_start;

    double time_accum = 0.0;
    for (size_t i = 0; i < N; ++i)
    {
        double target_time = i / frequency; // time in seconds
        while (time_accum < target_time)
        {
            size_t next_idx = (idx + 1) % reference_trajectory_.x.size();
            double dx = reference_trajectory_.x[next_idx] - reference_trajectory_.x[idx];
            double dy = reference_trajectory_.y[next_idx] - reference_trajectory_.y[idx];
            double segment_dist = std::sqrt(dx * dx + dy * dy);
            double segment_time = segment_dist / reference_trajectory_.v[idx];
            time_accum += segment_time;
            idx = next_idx;
        }

        x_ref_step[i] = reference_trajectory_.x[idx];
        y_ref_step[i] = reference_trajectory_.y[idx];
        psi_ref[i] = reference_trajectory_.yaw[idx];
        v_ref[i] = reference_trajectory_.v[idx];
    }

    // Publish the reference path for visualization
    nav_msgs::msg::Path ref_path;
    ref_path.header.stamp = this->get_clock()->now();
    ref_path.header.frame_id = frame_id_;
    for (size_t i = 0; i < N; ++i)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = ref_path.header;
        pose.pose.position.x = x_ref_step[i];
        pose.pose.position.y = y_ref_step[i];
        tf2::Quaternion q;
        q.setRPY(0, 0, psi_ref[i]);
        pose.pose.orientation = tf2::toMsg(q);
        ref_path.poses.push_back(pose);
    }
    ref_path_pub_->publish(ref_path);

    // 5. Set up the ACADOS solver
    for (size_t k = 0; k < N; k++)
    {
        std::vector<double> yref_k = {
            x_ref_step[k],
            y_ref_step[k],
            std::sin(psi_ref[k]),
            std::cos(psi_ref[k]),
            0.0,
            v_ref[k]};
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, k, "y_ref", yref_k.data());
    }

    // set yref for the last stage
    std::vector<double> yref_N = {
        x_ref_step[9],
        y_ref_step[9],
        std::sin(psi_ref[9]),
        std::cos(psi_ref[9])};
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N, "y_ref", yref_N.data());
}

bool MPCNode::load_reference_trajectory_from_csv(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
        return false;
    }

    std::string line;
    std::getline(file, line); 
    // Check if the first line is a header
    if (line.find("x,y,yaw,v") == std::string::npos)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid file format. Expected header: x,y,yaw,v");
        return false;
    }


    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string x_str, y_str, yaw_str, v_str;

        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, yaw_str, ',');
        std::getline(ss, v_str, ',');

        try
        {
            reference_trajectory_.x.push_back(std::stod(x_str));
            reference_trajectory_.y.push_back(std::stod(y_str));
            reference_trajectory_.yaw.push_back(std::stod(yaw_str));
            reference_trajectory_.v.push_back(std::stod(v_str));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error parsing line: %s. Exception: %s", line.c_str(), e.what());
            return false;
        }
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Loaded reference trajectory %s.", filename.c_str());
    return true;
}

void MPCNode::publish_reference_trajectory()
{
    nav_msgs::msg::Path ref_path;
    ref_path.header.stamp = this->get_clock()->now();
    ref_path.header.frame_id = frame_id_;
    for (size_t i = 0; i < reference_trajectory_.x.size(); ++i)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = ref_path.header;
        pose.pose.position.x = reference_trajectory_.x[i];
        pose.pose.position.y = reference_trajectory_.y[i];
        tf2::Quaternion q;
        q.setRPY(0, 0, reference_trajectory_.yaw[i]);
        pose.pose.orientation = tf2::toMsg(q);
        ref_path.poses.push_back(pose);
    }
    reference_trajectory_pub_->publish(ref_path);
}

void MPCNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    
    current_state_.v = msg->twist.twist.linear.x;

    if (!use_pose_topic_)
    {
        //ProcessPose(msg->pose.pose);
     

    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;
    
    geometry_msgs::msg::TransformStamped map_to_odom;
    try
    {
        map_to_odom = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform 'map' to 'odom': %s", ex.what());
        return;
    }

    geometry_msgs::msg::PoseStamped map_pose;
    tf2::doTransform(odom_pose, map_pose, map_to_odom);

    current_state_.x = map_pose.pose.position.x;
    current_state_.y = map_pose.pose.position.y;
    tf2::Quaternion q(
        map_pose.pose.orientation.x,
        map_pose.pose.orientation.y,
        map_pose.pose.orientation.z,
        map_pose.pose.orientation.w);
    current_state_.yaw = tf2::getYaw(q); 
    } 
       
}

void MPCNode::PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    ProcessPose(msg->pose);
}

void MPCNode::ProcessPose(const geometry_msgs::msg::Pose& msg)
{
    current_state_.x = msg.position.x;
    current_state_.y = msg.position.y;

    tf2::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);
    current_state_.yaw = tf2::getYaw(q);
}

void MPCNode::VescServoCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Extract servo position from VESC message
    current_state_.theta = (msg->data - steering_angle_to_servo_offset) / steering_angle_to_servo_gain;
    // RCLCPP_INFO(this->get_logger(), "Current servo position: %f", current_state_.theta);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Criação do nó
    auto mpc_node = std::make_shared<MPCNode>();

    rclcpp::spin(mpc_node);
    rclcpp::shutdown();
    return 0;
}