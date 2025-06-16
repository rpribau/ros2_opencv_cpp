#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "yolov8.h"
#include <string>
#include <vector>

class DetectionSubscriber : public rclcpp::Node {
public:
    DetectionSubscriber() : Node("detection_subscriber") {
        // --- Declaración y obtención de parámetros ---
        // Se declara el parámetro 'model_path'. Es obligatorio.
        this->declare_parameter<std::string>("model_path", "");
        // Se declara el parámetro 'model_type' con valor por defecto "object".
        this->declare_parameter<std::string>("model_type", "object");

        // Se obtienen los valores de los parámetros.
        std::string onnxModelPath = this->get_parameter("model_path").as_string();
        std::string modelType = this->get_parameter("model_type").as_string();

        // Verificación de que el path del modelo fue proporcionado.
        if (onnxModelPath.empty()) {
            RCLCPP_ERROR(this->get_logger(), "¡Parámetro 'model_path' no especificado! Este es obligatorio. Abortando.");
            rclcpp::shutdown();
            return;
        }

        // --- Configuración de YOLOv8 basada en parámetros ---
        YoloV8Config config;
        std::string trtModelPath = ""; // Opcional

        if (modelType == "pose") {
            RCLCPP_INFO(this->get_logger(), "Configurando para modelo de POSE.");
            config.classNames = {"person"}; // Para pose, solo hay una clase.
            config.isPose = true;
        } else if (modelType == "object") {
            RCLCPP_INFO(this->get_logger(), "Configurando para modelo de DETECCIÓN DE OBJETOS.");
            // No se necesita hacer nada, 'config' ya tiene por defecto las 80 clases de COCO.
        } else {
            RCLCPP_ERROR(this->get_logger(), "Tipo de modelo no válido: '%s'. Usa 'pose' o 'object'. Abortando.", modelType.c_str());
            rclcpp::shutdown();
            return;
        }

        // --- Inicialización del motor de YOLOv8 ---
        try {
            RCLCPP_INFO(this->get_logger(), "Creando instancia de YoloV8... Esto puede tardar varios minutos la primera vez.");
            yoloV8 = std::make_unique<YoloV8>(onnxModelPath, trtModelPath, config);
            RCLCPP_INFO(this->get_logger(), "Instancia de YoloV8 creada con éxito.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error al inicializar YOLOv8: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // --- Suscriptor y Publicador ---
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", qos_profile, std::bind(&DetectionSubscriber::topic_callback, this, std::placeholders::_1));
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/detection/image_processed", 10);

        RCLCPP_INFO(this->get_logger(), "Nodo de detección listo. Publicando resultados en /detection/image_processed");
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Callback recibido. Procesando frame.");
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_ptr->image;
            if (img.empty()) { 
                RCLCPP_WARN(this->get_logger(), "Frame de imagen vacío recibido.");
                return; 
            }

            RCLCPP_INFO(this->get_logger(), "Detectando objetos...");
            const auto objects = yoloV8->detectObjects(img);
            RCLCPP_INFO(this->get_logger(), "Detectados %zu objetos. Dibujando etiquetas.", objects.size());

            yoloV8->drawObjectLabels(img, objects);
            RCLCPP_INFO(this->get_logger(), "Etiquetas dibujadas. Publicando imagen procesada.");

            sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
            processed_msg->header = msg->header; // Conservar la cabecera original
            image_publisher_->publish(*processed_msg);
            RCLCPP_INFO(this->get_logger(), "Imagen procesada publicada.");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error en el callback: %s", e.what());
        }
    }

    std::unique_ptr<YoloV8> yoloV8;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectionSubscriber>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
