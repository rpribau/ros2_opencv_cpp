#!/usr/bin/env python3

# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import Int32
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import os
import yaml

def focal_length_finder(measured_distance, real_width, width_in_frame):
    focal_length = (width_in_frame * measured_distance) / real_width
    return focal_length

def distance_finder(focal_length, real_width, width_in_frame):
    if width_in_frame == 0:
        return 0
    distance = (real_width * focal_length) / width_in_frame
    return distance

class PeopleDistanceDetection(Node):
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('people_distance_detection')
        pkg_share_directory = get_package_share_directory('sdv_vision')

        # PARAMETERS
        self.declare_parameter('detection_mode', 'detection')  # Detection mode (calibration or detection)
        self.declare_parameter('calibration_distance', 2.0)  # Calibration distance in meters
        self.declare_parameter('calibration_person_width', 0.38)  # Width of person in meters
        self.declare_parameter('caution_distances', [1.5, 3.0])  # Distance thresholds for caution zones

        # CALIBRATION VALUES
        self.CALIBRATION_KNOWN_DISTANCE = self.get_parameter('calibration_distance').get_parameter_value().double_value
        self.CALIBRATION_PERSON_WIDTH = self.get_parameter('calibration_person_width').get_parameter_value().double_value
        self.focal_person = 426.3157894736842  # Default focal length
        self.FOCAL_PATH = os.path.join(pkg_share_directory, 'focal_person.yaml')

        # CAUTION DISTANCES
        self.caution_distances = self.get_parameter('caution_distances').get_parameter_value().double_array_value

        # TOPICS - SUBSCRIBERS
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Image, '/multisense/left/image_color', self.listener_callback, qos_profile)

        # TOPICS - PUBLISHERS
        self.publisher_video = self.create_publisher(Image, '/people_distance_detection_video', 10)
        self.pub_flag = self.create_publisher(Int32, '/people_distance_detections_flag', 10)
        self.flag_detection = Int32()

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def save_focal_person(self):
        """Save the focal person value to a YAML file."""
        with open(self.FOCAL_PATH, 'w') as file:
            self.get_logger().info(str(self.focal_person))
            yaml.dump({'focal_person': self.focal_person}, file)
            self.get_logger().info('Focal person value saved to focal_person.yaml')

    def listener_callback(self, data):
        # Get the detection mode from the parameter server (calibration or detection)
        mode = self.get_parameter('detection_mode').get_parameter_value().string_value

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # Example fixed values for person width and distance for testing without YOLO
        person_width = 0.3  # Substitute actual width detection with a default/fixed value

        if mode == "calibration":
            self.focal_person = focal_length_finder(self.CALIBRATION_KNOWN_DISTANCE, self.CALIBRATION_PERSON_WIDTH, person_width)
            self.save_focal_person()
            self.flag_detection.data = 4  # Calibration mode flag
        elif mode == "detection":
            distance = distance_finder(self.focal_person, self.CALIBRATION_PERSON_WIDTH, person_width)
            distance = round(float(distance), 1)
            if distance < self.caution_distances[0]:  # Danger zone
                self.flag_detection.data = 1
            elif self.caution_distances[0] <= distance < self.caution_distances[1]:  # Warning zone
                self.flag_detection.data = 2
            else:  # Safe zone
                self.flag_detection.data = 3

        # Publish flag and frame
        self.pub_flag.publish(self.flag_detection)
        self.publisher_video.publish(self.br.cv2_to_imgmsg(current_frame, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    people_distance_detection = PeopleDistanceDetection()
    rclpy.spin(people_distance_detection)
    people_distance_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
