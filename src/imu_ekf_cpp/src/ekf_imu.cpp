#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"

class OdomToImu : public rclcpp::Node
{
public:
    OdomToImu() : Node("odom_to_imu")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&OdomToImu::odomCallback, this, std::placeholders::_1));

        acc_sub_ = this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
            "/accel/filtered", 10,
            std::bind(&OdomToImu::accCallback, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/ekf", 10);

        imu_msg_ = std::make_shared<sensor_msgs::msg::Imu>();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50Hz = 20ms
            [this]()
            {
                if (imu_msg_)
                {
                    imu_pub_->publish(*imu_msg_);
                }
              
            });

        timer_print_status_ = this->create_wall_timer(
            std::chrono::seconds(5), // Print status every second
            [this]()
            {
                RCLCPP_INFO(this->get_logger(), "Received ACC messages: %.2f hz, Odometry messages: %.2f hz",
                            rcv_acc_count_/5.0, rcv_odom_count_/5.0);
                rcv_acc_count_= 0;
                rcv_odom_count_ = 0;
            });
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        rcv_odom_count_++;

        imu_msg_->orientation = msg->pose.pose.orientation;

        imu_msg_->angular_velocity = msg->twist.twist.angular;

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                imu_msg_->orientation_covariance[i * 3 + j] = msg->pose.covariance[(i + 3) * 6 + (j + 3)];
                imu_msg_->angular_velocity_covariance[i * 3 + j] = msg->twist.covariance[(i + 3) * 6 + (j + 3)];
            }
        }
    }

    void accCallback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
    {
        rcv_acc_count_++;

        imu_msg_->header = msg->header;

        imu_msg_->linear_acceleration = msg->accel.accel.linear;
        
        for (int i = 0; i < 9; ++i)
        {
            imu_msg_->linear_acceleration_covariance[i] = msg->accel.covariance[i];
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acc_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_print_status_;
    int rcv_acc_count_ = 0;
    int rcv_odom_count_ = 0;

    std::shared_ptr<sensor_msgs::msg::Imu> imu_msg_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToImu>());
    rclcpp::shutdown();
    return 0;
}
