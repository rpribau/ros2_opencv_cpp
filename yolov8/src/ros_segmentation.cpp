#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "cmd_line_util.h"
#include "yolov8.h"
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/imgproc.hpp>

class YoloV8Node : public rclcpp::Node
{
public:
    YoloV8Node() : Node("yolov8_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("model_path", "");
        this->declare_parameter<std::string>("camera_topic", "/camera/image_raw");
        this->declare_parameter<int>("image_width", 640);
        this->declare_parameter<int>("image_height", 480);
        this->declare_parameter<bool>("enable_resize", true);
        
        // Get parameters
        std::string model_path = this->get_parameter("model_path").as_string();
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();
        enable_resize_ = this->get_parameter("enable_resize").as_bool();

        // Initialize YOLOv8 configuration
        YoloV8Config config;
        
        try {
            // Create the YOLOv8 engine
            yolo_ = std::make_unique<YoloV8>(model_path, config);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize YOLOv8: %s", e.what());
            throw;
        }

        // Create publisher for the processed image
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>
            ("/yolov8/detected_objects", 10);

        // Create subscription with a larger queue size
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 
            10,
            std::bind(&YoloV8Node::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), 
            "YOLOv8 node initialized with image size: %dx%d", 
            image_width_, image_height_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS Image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat img = cv_ptr->image;

            if (img.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Received empty image");
                return;
            }

            cv::Mat resized_img;
            if (enable_resize_) {
                // Resize image to specified dimensions
                cv::resize(img, resized_img, cv::Size(image_width_, image_height_), 
                          0, 0, cv::INTER_LINEAR);
            } else {
                resized_img = img;
            }

            // Run inference
            const auto objects = yolo_->detectObjects(resized_img);

            // Draw the bounding boxes on the image
            yolo_->drawObjectLabels(resized_img, objects);

            // Convert back to ROS message and publish
            sensor_msgs::msg::Image::SharedPtr out_msg = 
                cv_bridge::CvImage(msg->header, "bgr8", resized_img).toImageMsg();
            publisher_->publish(*out_msg);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Runtime error: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    std::unique_ptr<YoloV8> yolo_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int image_width_;
    int image_height_;
    bool enable_resize_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YoloV8Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}