cmake_minimum_required(VERSION 3.5)
project(people_detection)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
)

# Add executable
add_executable(people_detection_node
    src/people_detection_node.cpp
    src/YoloPose.cpp
    src/ImageTools.cpp
    src/yolo_pose.cpp
)

# Link against libraries
ament_target_dependencies(people_detection_node
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
)

# Install executable
install(TARGETS
    people_detection_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
    DESTINATION include
)

# Install launch files if needed
# install(DIRECTORY
#     launch
#     DESTINATION share/${PROJECT_NAME}
# )

ament_package()
