from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from ament_index_python.packages import get_package_share_directory
# subprocess será importado dentro de OpaqueFunction

def generate_launch_description():
    # --- Argumentos de Lanzamiento Existentes ---
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        description='Ruta absoluta al archivo del modelo .onnx.'
    )

    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='object',
        description='Tipo de modelo a usar: "object" o "pose".'
    )

    sensor_arg = DeclareLaunchArgument(name='sensor',
                                   default_value='S21',
                                   description='Type of multisense: S21, SL, S7, S7S, S27, S30, KS21')

    namespace_arg = DeclareLaunchArgument(name='namespace',
                                      default_value='multisense',
                                      description='Namespace for this MultiSense instance')

    mtu_arg = DeclareLaunchArgument(name='mtu',
                                default_value='1500',
                                description='Sensor MTU')

    ip_address_arg = DeclareLaunchArgument(name='ip_address',
                                       default_value='10.66.171.21',
                                       description='Sensor IP address')

    launch_robot_state_publisher_arg = DeclareLaunchArgument(name='launch_robot_state_publisher',
                                                         default_value='True',
                                                         description='Launch the robot_state_publisher')

    multisense_image_topic_arg = DeclareLaunchArgument(
        'multisense_image_topic',
        default_value=[LaunchConfiguration('namespace'), '/left/image_raw'], # Ejemplo: multisense/left/image_raw
        description='El tópico de imagen de la MultiSense para la detección.'
    )

    # --- OpaqueFunction para decidir la cámara y lanzar nodos ---
    def decide_camera_and_launch(context):
        actions = []
        use_multisense = False

        # Obtener LaunchConfigurations para pasarlas a los Nodos
        lc_namespace = LaunchConfiguration('namespace')
        lc_ip_address = LaunchConfiguration('ip_address')
        lc_mtu = LaunchConfiguration('mtu')
        lc_sensor = LaunchConfiguration('sensor')
        lc_launch_rsp = LaunchConfiguration('launch_robot_state_publisher')
        lc_model_path = LaunchConfiguration('model_path')
        lc_model_type = LaunchConfiguration('model_type')
        lc_multisense_image_topic = LaunchConfiguration('multisense_image_topic')

        # --- Lógica para determinar model_type ---
        resolved_model_path = context.perform_substitution(lc_model_path)
        final_model_type = context.perform_substitution(lc_model_type)
        
        if 'pose' in resolved_model_path and final_model_type == 'object':
            final_model_type = 'pose'
            actions.append(LogInfo(msg=f"El path del modelo contiene 'pose'. Se forzará model_type='pose'."))

        # Obtener valor resuelto de la IP para la lógica de Python (ping)
        resolved_multisense_ip = context.perform_substitution(lc_ip_address) # CORREGIDO AQUÍ

        try:
            import subprocess
            # Comando ping para Linux, con timeout de 1s para respuesta, 2s para el proceso total.
            ping_command = ['ping', '-c', '1', '-W', '1', resolved_multisense_ip]
            result = subprocess.run(ping_command, capture_output=True, text=True, timeout=2)

            if result.returncode == 0:
                use_multisense = True
                actions.append(LogInfo(msg=f"Cámara MultiSense detectada en {resolved_multisense_ip}. Inicializando nodos de MultiSense."))
            else:
                actions.append(LogInfo(msg=f"Cámara MultiSense no encontrada en {resolved_multisense_ip} (ping falló). Intentando usar cámara de laptop."))
        except FileNotFoundError:
            actions.append(LogInfo(msg=f"Comando 'ping' no encontrado. No se pudo verificar la MultiSense. Intentando usar cámara de laptop."))
        except subprocess.TimeoutExpired:
            actions.append(LogInfo(msg=f"Timeout al intentar hacer ping a la MultiSense en {resolved_multisense_ip}. Intentando usar cámara de laptop."))
        except Exception as e:
            actions.append(LogInfo(msg=f"Error al verificar la cámara MultiSense en {resolved_multisense_ip}: {str(e)}. Intentando usar cámara de laptop."))
        
        if use_multisense:
            actions.extend([
                Node(package='yolov8_tensorrt_cpp',
                     executable='detection_subscriber',
                     name='detection_subscriber',
                     parameters=[{'model_path': lc_model_path, 'model_type': final_model_type}],
                     remappings=[('image_raw', lc_multisense_image_topic)]),
                Node(package='multisense_ros',
                     namespace=lc_namespace,
                     executable='ros_driver',
                     parameters=[{'sensor_ip': lc_ip_address,
                                  'sensor_mtu': lc_mtu,
                                  'tf_prefix': lc_namespace}]),
                Node(package='robot_state_publisher',
                     executable='robot_state_publisher',
                     namespace=lc_namespace,
                     condition=IfCondition(lc_launch_rsp),
                     parameters=[{'robot_description': Command([
                                 PathJoinSubstitution([FindPackagePrefix('xacro'), 'bin', 'xacro ']),
                                 PathJoinSubstitution([get_package_share_directory('multisense_ros'),
                                                       'urdf', lc_sensor, 'standalone.urdf.xacro']),
                                 " name:=", lc_namespace])}])
            ])
        else:
            # Este bloque se ejecuta si use_multisense es False (ping falló, error, o comando no encontrado)
            actions.append(LogInfo(msg="Inicializando cámara de laptop como alternativa."))
            actions.extend([
                Node(package='yolov8_tensorrt_cpp',
                     executable='detection_subscriber',
                     name='detection_subscriber',
                     parameters=[{'model_path': lc_model_path, 'model_type': final_model_type}]),
                Node(package='camera_publisher',
                     executable='webcam_node',
                     name='camera_publisher')
                     # Por defecto se suscribe a /image_raw que publicará webcam_node
            ])
        return actions

    opaque_function_action = OpaqueFunction(function=decide_camera_and_launch)

    return LaunchDescription([
        model_path_arg,
        model_type_arg,
        sensor_arg,
        namespace_arg,
        mtu_arg,
        ip_address_arg,
        launch_robot_state_publisher_arg,
        multisense_image_topic_arg,
        opaque_function_action
    ])

