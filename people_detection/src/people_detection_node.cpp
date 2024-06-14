#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "people_detection/main.h"

class PeopleDetectionNode : public rclcpp::Node
{
public:
    PeopleDetectionNode() : Node("people_detection_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&PeopleDetectionNode::imageCallback, this, std::placeholders::_1));

        // Inicializar YOLO una vez fuera del bucle principal
        yolo_ = std::make_shared<YoloPose>();
        yolo_->init("/home/rob/vanttec_sdv/workspace/src/sdv_vision/yolov8_object_detection/people_distance_detection/__sampledata/yolov8n-pose.onnx");

        // Inicializar el publisher para las detecciones de objetos
        pub_detection_ = this->create_publisher<sensor_msgs::msg::Image>("object_detection", 10);
    }

private:
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convertir el mensaje de imagen a una cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "Error al convertir mensaje de ROS a cv::Mat: %s", e.what());
        return;
    }

    // Procesar la imagen con YOLO
    yoloPose(cv_ptr->image, *yolo_);

    // Publicar la imagen con detecciones en el topic "object_detection"
    pub_detection_->publish(*cv_ptr->toImageMsg());

    // Opcional: Mostrar la imagen procesada si es necesario
    // cv::imshow("People Detection", cv_ptr->image);
    // cv::waitKey(1);
}

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::shared_ptr<YoloPose> yolo_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_detection_;  // Definición del publisher
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PeopleDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
