#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("cpp_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("vision_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&ImagePublisher::timer_callback, this));
        cap_.open(0); // Abre la cámara predeterminada
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cap_ >> frame; // Captura un fotograma de la cámara

        if (!frame.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}