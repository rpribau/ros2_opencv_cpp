#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def _init_(self):
        super()._init_('camera_node')
        
        # Declarar parámetros
        self.declare_parameter('camera_id', 0)  # 0 es generalmente la cámara web integrada
        self.declare_parameter('frame_rate', 30.0)  # FPS
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # Obtener parámetros
        self.camera_id = self.get_parameter('camera_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        # Crear el publicador
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )
        
        # Inicializar la cámara
        self.cap = cv2.VideoCapture(self.camera_id)
        
        # Configurar resolución
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        # Verificar si la cámara se abrió correctamente
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara')
            return
            
        # Crear bridge para convertir entre OpenCV y ROS
        self.bridge = CvBridge()
        
        # Crear timer para publicar frames
        period = 1.0 / self.frame_rate
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.get_logger().info(
            f'Nodo de cámara iniciado\n'
            f'ID de cámara: {self.camera_id}\n'
            f'Resolución: {self.image_width}x{self.image_height}\n'
            f'FPS: {self.frame_rate}'
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Directly use the original BGR frame from OpenCV
            # No color conversion needed
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            
            # Publicar mensaje
            self.publisher.publish(msg)
        else:
            self.get_logger().warn('No se pudo leer frame de la cámara')

    def _del_(self):
        # Liberar la cámara cuando se destruye el nodo
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Limpiar
        camera_node.destroy_node()
        rclpy.shutdown()

if _name_ == '_main_':
    main()
