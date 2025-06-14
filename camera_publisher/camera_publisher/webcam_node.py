import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        timer_period = 0.03  # seconds (aproximadamente 30 FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara.")
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # La línea siguiente convierte la imagen de OpenCV (Mat) a un mensaje de ROS (Image)
            # y la publica en el tópico 'image_raw'.
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
            self.get_logger().info('Publicando frame de video')
        else:
            self.get_logger().warn('No se pudo leer el frame de la cámara.')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.cap.release()
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
