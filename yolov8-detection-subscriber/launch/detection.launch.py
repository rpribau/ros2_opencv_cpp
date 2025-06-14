from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Declaración de Argumentos de Lanzamiento ---
    # El usuario podrá definir estos valores desde la línea de comandos.

    # Argumento para la ruta del modelo. Es obligatorio.
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        description='Ruta absoluta al archivo del modelo .onnx.'
    )

    # Argumento para el tipo de modelo. Por defecto es 'object'.
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='object',
        description='Tipo de modelo a usar: "object" o "pose".'
    )

    # --- Definición de Nodos ---

    # 1. Nodo de la cámara
    camera_node = Node(
        package='camera_publisher',
        executable='webcam_node',
        name='camera_publisher'
    )

    # 2. Nodo de detección YOLOv8
    detection_node = Node(
        package='yolov8_tensorrt_cpp',
        executable='detection_subscriber',
        name='detection_subscriber',
        # Pasa los argumentos de lanzamiento como parámetros al nodo.
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'model_type': LaunchConfiguration('model_type')
        }]
    )

    # --- Composición de la Descripción de Lanzamiento ---
    # Se devuelven todos los componentes para que ROS2 los ejecute.
    return LaunchDescription([
        model_path_arg,
        model_type_arg,
        camera_node,
        detection_node
    ])
