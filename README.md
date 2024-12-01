# ros2_opencv_cpp

**Este repo son meras pruebas que voy haciendo de tiempo en tiempo para el SDV para [@Vanttec](https://github.com/vanttec). El codigo no va ser el mismo que tenia hace uno o dos meses, por lo que si quieres ver versiones anteriores buscalas por los commits.**

**Nota del momento**: frfr on god, que la vision si jala en la Jetson. desafortunadamente no puse atencion a mi clase de Packet Tracer y no se de redes xDDDDDDDD.

<p align="center">
  <img src="https://preview.redd.it/lsdzxdt28f161.png?auto=webp&s=1aaa635d73c7334b54239fb5dc1a57e0fa2279f7" alt="Imagen centrada" width="600"/>
</p>


## Introduccion

Estos son los avances mas grandes que he hecho en los ultimos dos meses, los cuales se conformaron en saber como instalar bien OpenCV con CUDA, instalar TensorRT, tener bien los drivers de NVIDIA porque los que vienen por default en Ubuntu son basura y sobretodo, que funcione la cosa. Podran notar que elimine muchisimos archivos anteriores de mis pruebas, esto debido a que mi logica de como hacer las cosas no funcionaba muy bien en la practica, no compilaba bien las builds usando ```colcon build``` y sobre todo la pelea fue mas con TensorRT.

## Pasos de instalación de OpenCV (Jetson/Local Machine)

Voy a asumir que ya esta lo siguiente instalado por JetPack:

**Requerimientos**
- Ubuntu 22.04
- ROS2 Humble
- CUDA >= 11.8
- cuDNN >= 8.6
- TensorRT >= 8.6
- gcc = 11.4
- g++ = 11.4

### Instalar OpenCV

#### Instalar los siguientes CODECS (no vienen por default en la Jetson, quiza en tu lap si vengan)

Camara y Media Support (video) (ffmpeg, gstreamer…)
```
sudo apt install libavcodec-dev libavformat-dev libswscale-dev
sudo apt install libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
sudo apt install libgtk-3-dev
```

Formatos extras (esta es opcional, pero igual pueden ser utiles a futuro)
```
sudo apt install libpng-dev libjpeg-dev libopenexr-dev libtiff-dev libwebp-dev
```

#### Descargar OpenCV y buildear

```
git clone https://github.com/opencv/opencv.git && git clone https://github.com/opencv/opencv_contrib.git
```

Entra a la carpeta de OpenCV, crea la carpeta build y accede a esta.
```
cd opencv && mkdir build && cd build
```

Busca la capacidad computacional de la Jetson en la siguiente pagina: https://developer.nvidia.com/cuda-gpus. Una vez encontrada, ejecuta el siguiente comando:

```
cmake \
-D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D WITH_CUBLAS=ON \
-D WITH_TBB=ON \
-D OPENCV_DNN_CUDA=ON \
-D OPENCV_ENABLE_NONFREE=ON \
-D CUDA_ARCH_BIN=6.1 \         <--- Edita este numero por la capacidad de la Jetson.
-D OPENCV_EXTRA_MODULES_PATH=$HOME/opencv_contrib/modules \
-D BUILD_EXAMPLES=OFF \
-D HAVE_opencv_python3=ON \
..
```

La terminal deberia de decirte que version de OpenCV estas instalando junto con las especificaciones del CUDA y cuDNN.

```
...
-- The CXX compiler identification is GNU 9.4.0
...
-- Performing Test......
-- General configuration for OpenCV 4.10.0-dev ======================
...
--   NVIDIA CUDA:                   YES (ver 11.8, CUFFT CUBLAS)
--     NVIDIA GPU arch:             61
--     NVIDIA PTX archs:
-- 
--   cuDNN:                         YES (ver 8.6.0)...-- Configuring done
-- Generating done
-- Build files have been written to: /home/rob/opencv/build
```

Una vez que aparezca el Generating done, corre el siguiente comando

```
make -j 2
```

```-j``` es el numero de nucleos de procesador quieres usar para buildear OpenCV, si lo estas buildeando desde la Jetson, yo sugiero que solo uses 2 nucleos. Entre mas nucleos, mas rapido buildea pero con un precio de que el sistema se crashea y tengas que reiniciar todo. Este proceso puede tardar mas o menos 1 o 2 horas si lo especificas con 2 nucleos.

Una vez termiando de buildear, ejecuta el siguiente comando

```
sudo make install
```

Terminado esto solo corre el siguiente comando:
```
sudo ldconfig
```

Y listo! En teoria deberia de estar instalado OpenCV con CUDA. Una forma para verificar es abriendo ```python3``` y correr las siguientes lineas

```
import cv2
cv2.cuda.printCudaDeviceInfo(0)
```

## Configurar proyecto

Antes de clonar el proyecto, crea desde Home la siguientes carpetas.

```
mkdir ros2_ws/src
```

**Nota. Es importante que sea ```ros2_ws```, si no la Multisense no funcionara**

Clona el proyecto desde la carpeta source con la siguiente linea:

```
git clone https://github.com/rpribau/ros2_opencv_cpp .
```

Dentro de src clona la el siguiente repositorio de ```xacro``` e instala las siguientes dependencias de ROS

```
git clone -b dashing-devel https://github.com/ros/xacro.git
sudo apt install ros-humble-xacro ros-humble-tf2-geometry-msgs
```

Mueve el archivo de ```yolov8n-pose.onnx``` y ```multisense.sh``` a ros2_ws/

Y ya lo que queda es buildear y correr ```multisense.sh``` 

- Terminal 1:

```
colcon build
source install/setup.bash
sudo ./multisense.sh
ros2 launch multisense_ros multisense_launch.py
```

- Terminal 2:
```
source install/setup.bash
ros2 run yolov8 ros_segmentation --ros-args -p model_path:=/home/vanttec/ros2_ws/yolov8n-pose.onnx -p camera_topic:=/multisense/left/image_color
```

- Terminal 3
```
rqt
```



## ¿Como funciona todo el codigo?

### Topico 1. camara_publisher

```
camera_publisher/
├── camera_publisher
│   ├── camera_node.py
│   └── __init__.py
├── package.xml
├── resource
│   └── camera_publisher
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

```

```camara_node.py```: Esto es un nodo muy basico de Python donde solo activa la camara de mi computadora y crea el siguiente topico ```/camera/image_raw``` para que pueda usar la camara de rqt sin ningun problema y que tambien otro nodo se pueda suscribir y recibir la imagen.

### Topico 2. yolov8

```
.
├── cmake
│   ├── ccache.cmake
│   └── FindTensorRT.cmake
├── CMakeLists.txt
├── config
│   └── yolov8.rviz
├── launch
│   └── yolov8.launch.py
├── libs
│   └── tensorrt-cpp-api
│       ├── cmake
│       │   ├── ccache.cmake
│       │   └── FindTensorRT.cmake
│       ├── CMakeLists.txt
│       ├── images
│       │   └── logo.png
│       ├── inputs
│       │   └── team.jpg
│       ├── LICENSE
│       ├── models
│       ├── README.md
│       └── src
│           ├── cmd_line_parser.h
│           ├── engine.cpp
│           ├── engine.h
│           └── main.cpp
├── models
│   ├── engines
│   │   └── yolov8n-pose.NVIDIAGeForceMX450fp16maxBatchSize1optimalBatchSize1.engine
│   └── yolov8n-pose.onnx
├── package.xml
└── src
    ├── benchmark.cpp
    ├── cmd_line_util.h
    ├── ros_segmentation.cpp
    ├── yolov8.cpp
    └── yolov8.h

```

**Nota**: aqui se ignora la carpeta scripts porque hay se encuentra la instalación de OpenCV. No es relevante en este momento, sin embargo, deje un script para instalar OpenCV sin quedarte pelon en el intento.

La forma en como funciona se basa principalmente en la <a href="https://github.com/cyrusbehr/tensorrt-cpp-api">TensorRT C++ API de cyrusbehr</a> que la verdad es la cosa mas bonita que haya visto en mi vida. Lo que hace toda la parte de ```libs/tensorrt-cpp-api``` se encarga de crear el modelo de ONNX a un ```.engine``` para poder generar los modelos mas facilmente. 

Una vez que se generan los modelos desde la lib, podemos pasar directamente a ```src/```:

#### yolov8.cpp 

Soporte para Múltiples Tareas:

- Detección de Objetos
- Segmentación de Instancias
- Estimación de Posturas

Aceleración por GPU:

- Procesamiento habilitado por CUDA utilizando el módulo de GPU de OpenCV
- Optimización con TensorRT para una inferencia rápida
- Soporte para diferentes modos de precisión (FP32, FP16, INT8)

Pre/Post Procesamiento:

- Preprocesamiento eficiente basado en GPU
- Redimensionado que preserva la relación de aspecto
- Supresión de Máximos No Máximos (NMS)
- Umbrales configurables para confianza y NMS

Visualización:

- Dibujo de cajas delimitadoras
- Etiquetas de clase con puntuaciones de confianza
- Superposición de máscaras de segmentación
- Visualización de puntos clave con dibujo de esqueleto para estimación de posturas

#### yolov8.h

Precargar objetos necesarios

- Esto con el fin de poder agregar los objetos dentros disponibles dentro de nuestros modelos de YOLO. Al menos en el mio solo agregue 'person'.

Cargar el ```.engine``` y hacer detecciones

- Puede determinar que tipo de modelo de YOLO es, si es uno de segmentación, un modelo normal de detección de objetos o un modelo pose.

#### ros_segmentation.cpp

Aqui esta toda la magia, donde corre todo lo que vienen en los dos archivos anteriores. Lo unico que hice fue crear otro publisher ```/yolov8/detected_objects``` donde se manda la imagen a rqt.






