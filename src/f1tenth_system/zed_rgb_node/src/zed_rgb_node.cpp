#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "videocapture.hpp"

class ZedRGBPublisher : public rclcpp::Node
{
public:
    ZedRGBPublisher() : Node("zed_rgb_publisher")
    {
        // Parâmetros
        this->declare_parameter("camera_id", 0);
        this->declare_parameter("fps", 30);

        int cam_id = this->get_parameter("camera_id").as_int();
        int fps = this->get_parameter("fps").as_int();

        sl_oc::video::FPS fps_enum;
        switch (fps)
        {
        case 15:
            fps_enum = sl_oc::video::FPS::FPS_15;
            break;
        case 30:
            fps_enum = sl_oc::video::FPS::FPS_30;
            break;
        case 60:
            fps_enum = sl_oc::video::FPS::FPS_60;
            break;
        case 100:
            fps_enum = sl_oc::video::FPS::FPS_100;
            break;
        default:
            std::cerr << "FPS inválido, usando 30 por defeito." << std::endl;
            fps_enum = sl_oc::video::FPS::FPS_30;
        }

        sl_oc::video::VideoParams params;
        params.res = sl_oc::video::RESOLUTION::HD720;
        params.fps = fps_enum;


        //sl_oc::video::VideoCapture cap_0(params);
        //camera_ = std::make_unique<sl_oc::video::VideoCapture>(cap_0);

        camera_ = std::make_unique<sl_oc::video::VideoCapture>(params);



        // Inicializar câmara
        if (!camera_->initializeVideo(cam_id))
        {
            RCLCPP_ERROR(this->get_logger(), "Erro ao inicializar a ZED 2i.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to camera sn: %d [%s]", camera_->getSerialNumber(), camera_->getDeviceName().c_str());


         // Create ROS publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("zed2i/rgb/image_raw", 10);

        publisher_left_ = this->create_publisher<sensor_msgs::msg::Image>("zed2i/rgb/image_raw_left", 10);
        publisher_right_ = this->create_publisher<sensor_msgs::msg::Image>("zed2i/rgb/image_raw_right", 10);


        // Timer to periodically grab frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / fps),
            std::bind(&ZedRGBPublisher::timer_callback, this));
    }   

private:
    void timer_callback()
    {
        // Get last available frame
        const sl_oc::video::Frame frame = camera_->getLastFrame();
        if (frame.data == nullptr)
        {
            RCLCPP_WARN(this->get_logger(), "No frame available");
            return;
        }
        // ----> Conversion from YUV 4:2:2 to BGR for visualization
        cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
        cv::Mat frameBGR;
        cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

        // ----> Conversion from BGR to ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frameBGR).toImageMsg();
        msg->header.stamp = this->now();
        publisher_->publish(*msg); 

        // Separar os dois olhos
        int width = frameBGR.cols / 2;
        cv::Mat left = frameBGR(cv::Rect(0, 0, width, frameBGR.rows));
        cv::Mat right = frameBGR(cv::Rect(width, 0, width, frameBGR.rows));

        auto msg_left = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left).toImageMsg();
        msg_left->header.stamp = this->now();
        publisher_left_->publish(*msg_left);

        auto msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right).toImageMsg();
        msg_right->header.stamp = this->now();
        publisher_right_->publish(*msg_right);
    }

    std::unique_ptr<sl_oc::video::VideoCapture> camera_;  // Usar unique_ptr para gerenciar o objeto de câmera
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_, publisher_right_, publisher_left_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedRGBPublisher>());
    rclcpp::shutdown();
    return 0;
}
