cmake_minimum_required(VERSION 3.22)
project(yolov8_tensorrt_cpp)

# Encuentra paquetes de ROS2 y dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# TODO: Especifica la ruta a tu directorio raíz de TensorRT
set(TensorRT_DIR /home/cyrus/work/libs/TensorRT-10.0.0.6/) # ¡CAMBIA ESTO!

# Configuración de C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG -Wno-deprecated-declarations")

# Añade subdirectorio para la API de TensorRT
add_subdirectory(libs/tensorrt-cpp-api)

# Construye la librería de YoloV8
add_library(YoloV8_TRT SHARED src/yolov8.cpp)
target_link_libraries(YoloV8_TRT PUBLIC tensorrt_cpp_api ${OpenCV_LIBS})
target_include_directories(YoloV8_TRT PUBLIC libs/tensorrt-cpp-api/src)

# ---- NUEVO NODO DE ROS2 ----
# Construye el ejecutable del suscriptor de detección
add_executable(detection_subscriber src/detection_subscriber.cpp)

# Enlaza el nuevo nodo con la librería YoloV8_TRT y las dependencias de ROS2
target_link_libraries(detection_subscriber YoloV8_TRT)
ament_target_dependencies(detection_subscriber rclcpp sensor_msgs cv_bridge)

# ---- EJECUTABLES ORIGINALES (Opcional, los puedes mantener si quieres) ----
add_executable(detect_object_image src/object_detection_image.cpp)
target_link_libraries(detect_object_image YoloV8_TRT)
add_executable(benchmark src/benchmark.cpp)
target_link_libraries(benchmark YoloV8_TRT)
add_executable(detect_object_video src/object_detection_video_stream.cpp)
target_link_libraries(detect_object_video YoloV8_TRT)

# --- Reglas de Instalación de ROS2 ---
install(TARGETS
  detection_subscriber
  detect_object_image # Opcional
  benchmark           # Opcional
  detect_object_video # Opcional
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Esta función debe ir al final
ament_package()
