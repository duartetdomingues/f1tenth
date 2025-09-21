#include "mpc/mpc.hpp"

MPCNode::MPCNode() : Node("mpc_node"),
                     capsule(mpc_model_acados_create_capsule()),
                     speed_to_erpm_gain(4277.5),
                     speed_to_duty(0.0602),
                     steering_angle_to_servo_gain(-0.840),
                     steering_angle_to_servo_offset(0.475),
                     frequency(10.0),
                     tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
                     tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{

    RCLCPP_INFO(this->get_logger(), "MPC Node initialized.");
    // Declare parameters
    this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    this->declare_parameter<std::string>("traj_file", "./traj/centerline_v2_map_2025-09-09_10-52-29.csv");
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

    // Publishers 
    control_vesc_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("autonomous_control", 10);
    solved_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("mpc/solved_time", 10);
    ref_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc/ref_path", 10);
    reference_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc/ref_traj", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    simulation_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc/simulation_traj", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/state_array", 10);
    control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/control_array", 10);

    for (int i = 0; i < 2; i++)
        control_vector_pub_[i] = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/predict_u" + std::to_string(i), 10);

    for (int i = 0; i < 8; i++)
        state_vector_pub_[i] = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/predict_x" + std::to_string(i), 10);

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&MPCNode::OdomCallback, this, std::placeholders::_1));

    if (use_pose_topic_)
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 10, std::bind(&MPCNode::PoseCallback, this, std::placeholders::_1));
    }

    vesc_servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/vesc/servo_position_command", 10, std::bind(&MPCNode::VescServoCallback, this, std::placeholders::_1));

    vesc_state_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
        "/vesc/core", 10, std::bind(&MPCNode::VescStateCallback, this, std::placeholders::_1));

    // Load reference trajectory
    if (!load_reference_trajectory_from_csv(traj_file))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load reference trajectory.");
        rclcpp::shutdown();
    }
    // publish_reference_trajectory();

    kd_tree = prepare_kd_tree(reference_trajectory_.x, reference_trajectory_.y);

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
    /* std::vector<double> cost_weights = {10.0, 10.0, 0.0, 0.0, 1.0, 1.0}; // Example weights for x, y, sin(psi), cos(psi), steering, velocity
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(6, 6);
    W.diagonal() << cost_weights[0], cost_weights[1], cost_weights[2], cost_weights[3], cost_weights[4], cost_weights[5];
    for (size_t k = 0; k < MPC_MODEL_N; k++)
    {
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, k, "W", W.data());
    } */
    // Set final cost weights
    // std::vector<double> final_cost_weights = {1.0, 1.0, 0.0, 0.0}; // Example weights for x, y, sin(psi), cos(psi), steering, velocity
    /* std::vector<double> final_cost_weights = {10.0, 10.0, 1.0, 1.0}; // Example weights for x, y, sin(psi), cos(psi), steering, velocity
    Eigen::MatrixXd Wf = Eigen::MatrixXd::Zero(4, 4);
    Wf.diagonal() << final_cost_weights[0], final_cost_weights[1], final_cost_weights[2], final_cost_weights[3];
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, MPC_MODEL_N, "W", Wf.data()); */

    // nlp_config_ = ocp_nlp_config_create(nlp_dims_);
    // nlp_dims_ = ocp_nlp_dims_create(nlp_config_);
    // nlp_in_ = ocp_nlp_in_create(nlp_config_, nlp_dims_);
    // nlp_out_ = ocp_nlp_out_create(nlp_config_, nlp_dims_);
    // nlp_opts_ = ocp_nlp_solver_opts_create(nlp_config_, nlp_dims_);
    // nlp_solver_ = ocp_nlp_solver_create(nlp_config_, nlp_dims_, nlp_opts_, nlp_in_);

    // precompute
    // ocp_nlp_precompute(nlp_solver_, nlp_in_, nlp_out_);

    // Define weights and safety margin
    double weight_ds = 15.0;
    double weight_beta = 0.15;
    double weight_dalpha = 1.0;
    double weight_dthrottle = 1.0;
    double safety_margin = 0.1; // 10 cm

    // Parameter vector
    double p[7] = {weight_ds, weight_beta, weight_dalpha, weight_dthrottle, 1.0, 1.0, safety_margin};
    int idxs[7] = {0, 1, 2, 3, 4, 5, 6}; // indices for the parameters

    for (int i = 0; i < MPC_MODEL_N; i++)
    {
        ocp_nlp_in_set_params_sparse(nlp_config_, nlp_dims_, nlp_in_, i, idxs, p, 7);
    }

    // Set print level 0
    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "print_level", &print_level);

    // Init the states
    /* current_state_.s = 0;
    current_state_.n = 0.1;
    current_state_.u = 0.1;
    current_state_.vx = 0.1;
    current_state_.vy = 0.01;
    current_state_.r = 0.1;
    
    current_state_.T = 0.0; */

    n_x = 8; // Number of states
    n_u = 2; // Number of controls

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(100)),
        std::bind(&MPCNode::initMPC, this));
  
}

MPCNode::~MPCNode()
{
    delete kd_tree;
    mpc_model_acados_free(capsule);
}

void MPCNode::initMPC()
{
    current_state_.delta = 0.0;

    if ((std::isnan(current_state_.s) || std::isnan(current_state_.n) || std::isnan(current_state_.u) ||
            std::isnan(current_state_.vx) || std::isnan(current_state_.vy) || std::isnan(current_state_.r) ||
            std::isnan(current_state_.delta) || std::isnan(current_state_.T)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for valid initial state...");
        RCLCPP_INFO(this->get_logger(), "Current state: s=%.3f, n=%.3f, u=%.3f, vx=%.3f, vy=%.3f, r=%.3f, delta=%.3f, T=%.3f",
               current_state_.s, current_state_.n, current_state_.u, current_state_.vx,
               current_state_.vy, current_state_.r, current_state_.delta, current_state_.T);
    }
    else
    {
        timer_->cancel();
        
        current_state_.vx = 0.1;
        Eigen::VectorXd x0(8);
        x0 << current_state_.s, current_state_.n, current_state_.u, current_state_.vx,
            current_state_.vy, current_state_.r, current_state_.delta, current_state_.T;
        apply_warm_start(x0);

        // Set up a timer to solve MPC at a fixed frequency
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / frequency)),
            std::bind(&MPCNode::solveMPC, this));
    }


}

void MPCNode::solveMPC()
{
    if (MPC_MODEL_N <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid number of stages in MPC model.");
        return;
    }

    /* if (std::isnan(current_state_.s) || std::isnan(current_state_.n) || reference_trajectory_.x.empty() || reference_trajectory_.y.empty())
    {
        RCLCPP_WARN(this->get_logger(), "State or reference trajectory is empty. Skipping MPC solve.");
        return;
    } */

    // Get the reference trajectory to horizont
    // set_trajectory_step();

    // Update states
    // update_states_step();

    // Set initial state constraints
    std::vector<double> current_state_vector_ = {
        current_state_.s,
        current_state_.n,
        current_state_.u,
        current_state_.vx,
        current_state_.vy,
        current_state_.r,
        current_state_.delta,
        current_state_.T};

    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", current_state_vector_.data());
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", current_state_vector_.data());

    // Publish state array
    std_msgs::msg::Float64MultiArray state_array_msg;
    state_array_msg.data = {current_state_vector_[0],
                            current_state_vector_[1],
                            current_state_vector_[2],
                            current_state_vector_[3],
                            current_state_vector_[4],
                            current_state_vector_[5],
                            current_state_vector_[6],
                            current_state_vector_[7]};
    state_pub_->publish(state_array_msg);

    RCLCPP_INFO(this->get_logger(), "Current state: s=%.3f, n=%.3f, u=%.3f, vx=%.3f, vy=%.3f, r=%.3f, delta=%.3f, T=%.3f",
                current_state_.s, current_state_.n, current_state_.u, current_state_.vx,
                current_state_.vy, current_state_.r, current_state_.delta, current_state_.T);

    // Solve the MPC problem
    // int status = ocp_nlp_solve(nlp_solver_, nlp_in_, nlp_out_);
    int status = mpc_model_acados_solve(capsule);
    //  mpc_model_acados_print_stats(capsule);
    /* std::vector<double> xtraj((MPC_MODEL_N + 1) * 6);
    std::vector<double> utraj(MPC_MODEL_N * 2);
    for (int ii = 0; ii <= MPC_MODEL_N; ii++)
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj[ii*6]);
    for (int ii = 0; ii < MPC_MODEL_N; ii++)
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj[ii*2]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( 6, MPC_MODEL_N+1, xtraj.data(), 6);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( 2, MPC_MODEL_N, utraj.data(), 2 ); */

    // Get control output
    std::array<double, 8> state_output;
    double solved_time;

    if (status != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "\033[1;31mACADOS solver failed with status %d\033[0m", status);
        solved_time = -1.0;
        state_output[6] = current_state_.delta;
        state_output[7] = current_state_.T; // Maintain current control if solver fails
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32mACADOS solver succeeded with status %d\033[0m", status);

        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 1, "x", state_output.data()); // Retrieve control output

        RCLCPP_INFO(this->get_logger(), "Controls: delta=%.3f, T=%.3f", state_output[6], state_output[7]);

        ocp_nlp_get(nlp_solver_, "time_tot", &solved_time);
    }

    std_msgs::msg::Float64 solved_time_msg;
    solved_time_msg.data = solved_time;
    solved_time_pub_->publish(solved_time_msg);

    RCLCPP_INFO(this->get_logger(), "Solved time: %.2f milliseconds", solved_time * 1000);

    ackermann_msgs::msg::AckermannDriveStamped control_vesc_msg;
    control_vesc_msg.header.stamp = this->get_clock()->now();
    control_vesc_msg.header.frame_id = "base_link";
    control_vesc_msg.drive.steering_angle = state_output[6] * steering_angle_to_servo_gain;
    control_vesc_msg.drive.acceleration = state_output[7];

    // Publish control output
    control_vesc_pub_->publish(control_vesc_msg);

    // Publish control array
    std_msgs::msg::Float64MultiArray control_array_msg;
    control_array_msg.data = {state_output[6], state_output[7]};
    control_pub_->publish(control_array_msg);

    // Publish control vector
    /*std::vector<double> predict_vector_u0(MPC_MODEL_N), predict_vector_u1(MPC_MODEL_N);
    for (int ii = 0; ii < MPC_MODEL_N; ii++)
    {
        std::array<double, 2> control_vector;
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", control_vector.data());
        predict_vector_u0[ii] = control_vector[0];
        predict_vector_u1[ii] = control_vector[1];
    }
    std_msgs::msg::Float64MultiArray predict_u0_msg, predict_u1_msg;
    predict_u0_msg.data = predict_vector_u0;
    predict_u1_msg.data = predict_vector_u1;
    control_vector_pub_[0]->publish(predict_u0_msg);
    control_vector_pub_[1]->publish(predict_u1_msg); */

    // Publish state vector
    // Create a vector of 8 vectors, each with MPC_MODEL_N+1 elements
    std::vector<std::vector<double>> predict_vector_x(8, std::vector<double>(MPC_MODEL_N + 1));
    for (int ii = 0; ii <= MPC_MODEL_N; ii++)
    {
        std::array<double, 8> state_vector;
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", state_vector.data());
        // Fill predict_vector_x: [x, y, yaw, theta, v]
        for (int j = 0; j < 8; j++)
        {
            predict_vector_x[j][ii] = state_vector[j];
        }
    }

    std::vector<std_msgs::msg::Float64MultiArray> state_vector_msgs(8);
    for (int i = 0; i < 8; i++)
    {
        state_vector_msgs[i].data = predict_vector_x[i];
        state_vector_pub_[i]->publish(state_vector_msgs[i]);
    }

    /*  // Publish simulation trajectory
     nav_msgs::msg::Path simulation_path;
     simulation_path.header.stamp = this->get_clock()->now();
     simulation_path.header.frame_id = frame_id_;
     for (int ii = 0; ii <= MPC_MODEL_N; ii++)
     {
         geometry_msgs::msg::PoseStamped pose;
         pose.header = simulation_path.header;
         pose.pose.position.x = predict_vector_x[0][ii];
         pose.pose.position.y = predict_vector_x[1][ii];
         tf2::Quaternion q;
         q.setRPY(0, 0, predict_vector_x[2][ii]);
         pose.pose.orientation = tf2::toMsg(q);
         simulation_path.poses.push_back(pose);
     }
     simulation_trajectory_pub_->publish(simulation_path); */

    // RCLCPP_INFO(this->get_logger(), "Current steering angle: %f", current_state_.steering_angle);
}

/* void MPCNode::set_trajectory_step()
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
    std::vector<double> x_ref_step(MPC_MODEL_N), y_ref_step(MPC_MODEL_N), psi_ref(MPC_MODEL_N), v_ref(MPC_MODEL_N);
    size_t idx = idx_ref_start;

    double time_accum = 0.0;
    for (size_t i = 0; i < MPC_MODEL_N; ++i)
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
    for (size_t i = 0; i < MPC_MODEL_N; ++i)
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
    for (size_t k = 0; k < MPC_MODEL_N; k++)
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
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, MPC_MODEL_N, "y_ref", yref_N.data());
} */

void MPCNode::apply_warm_start(const Eigen::VectorXd &x)
{
    // get_warm_start devolve std::vector<Eigen::VectorXd>
    auto ws = this->get_warm_start(x, /*const_steer_vel=*/0.0, /*d_acc=*/0.05);

    for (int i = 0; i <= MPC_MODEL_N; i++)
    {
        // Parte de estado (primeiros n_x elementos)
        Eigen::VectorXd x_part = ws[i].head(this->n_x);
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x_part.data());

        // Parte de controle (resto do vetor), apenas se i < N
        if (i < MPC_MODEL_N)
        {
            Eigen::VectorXd u_part = ws[i].segment(this->n_x, ws[i].size() - this->n_x);
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", u_part.data());
        }
    }
}

std::vector<Eigen::VectorXd> MPCNode::get_warm_start(
    const Eigen::VectorXd &x,
    double d_acc,
    double const_steer_vel)
{
    double v_x_min_ws = 0.5; // [m/s] mínimo para evitar singularidades

    int horizon = MPC_MODEL_N + 1;
    int state_dim = 10; // TODO: hardcoded state space dimension

    std::vector<Eigen::VectorXd> warm_start(horizon, Eigen::VectorXd::Zero(state_dim));

    // inicialização (i = 0)
    warm_start[0] << x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7),
        const_steer_vel, d_acc;

    // Propagação simples
    for (int i = 1; i < horizon; i++)
    {
        // Eigen::VectorXd xdot = this->_dynamics_of_car(0.0, warm_start[i - 1]);

        // Integração de Euler explícito (comentado no Python)
        // warm_start[i] = warm_start[i - 1] + xdot / this->stmpc.MPC_freq;

        // No código atual apenas copia o anterior
        warm_start[i] = warm_start[i - 1];
    }

    return warm_start;
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
    if (line.find("s,x,y,kappa") == std::string::npos)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid file format. Expected header: s,x,y,kappa");
        return false;
    }

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string s_str, x_str, y_str, kappa_str;

        std::getline(ss, s_str, ',');
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, kappa_str, ',');

        try
        {
            reference_trajectory_.x.push_back(std::stod(x_str));
            reference_trajectory_.y.push_back(std::stod(y_str));
            reference_trajectory_.s.push_back(std::stod(s_str));
            reference_trajectory_.kappa.push_back(std::stod(kappa_str));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error parsing line: %s. Exception: %s", line.c_str(), e.what());
            return false;
        }
    }

    // Chama a função de integração acumulada
    cumtrapz(reference_trajectory_.s, reference_trajectory_.kappa, reference_trajectory_.psi);

    file.close();
    RCLCPP_INFO(this->get_logger(), "Loaded reference trajectory %s.", filename.c_str());
    return true;
}

void MPCNode::publish_reference_trajectory()
{
    /* nav_msgs::msg::Path ref_path;
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
    reference_trajectory_pub_->publish(ref_path); */
}

void MPCNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

    current_state_.vx = msg->twist.twist.linear.x;
    current_state_.vy = msg->twist.twist.linear.y;
    current_state_.r = msg->twist.twist.angular.z;

    /* if (!use_pose_topic_)
    {
        // ProcessPose(msg->pose.pose);

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
    } */
}

void MPCNode::PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    ProcessPose(msg->pose);
}

void MPCNode::ProcessPose(const geometry_msgs::msg::Pose &msg)
{
    double x = msg.position.x;
    double y = msg.position.y;

    tf2::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);
    double psi = tf2::getYaw(q);

    double X_s, Y_s, psi_s;

    global_to_local_pose(kd_tree->index, reference_trajectory_.s, reference_trajectory_.x,
                         reference_trajectory_.y, reference_trajectory_.psi,
                         x, y, psi,
                         current_state_.s, current_state_.n, current_state_.u,
                         X_s, Y_s, psi_s);
}

void MPCNode::VescServoCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Extract servo position from VESC message
    current_state_.delta = (msg->data - steering_angle_to_servo_offset) / steering_angle_to_servo_gain;
    // RCLCPP_INFO(this->get_logger(), "Current servo position: %f", current_state_.theta);
}

void MPCNode::VescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
    // Process VESC state message
    current_state_.T = msg->state.duty_cycle;
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