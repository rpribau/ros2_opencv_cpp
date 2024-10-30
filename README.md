# ros2_opencv_cpp

**Este repo son meras pruebas que voy haciendo de tiempo en tiempo para el SDV para [@Vanttec](https://github.com/vanttec). El codigo no va ser el mismo que tenia hace uno o dos meses, por lo que si quieres ver versiones anteriores buscalas por los commits.**

**Nota del momento**: Hoy amanecimos pelones.

<p align="center">
  <img src="https://i1.sndcdn.com/artworks-lb3oCCJzTzFKeADH-oy18gg-t500x500.jpg" alt="Imagen centrada" width="300"/>
</p>


## Introduccion

Estos son los avances mas grandes que he hecho en los ultimos dos meses, los cuales se conformaron en saber como instalar bien OpenCV con CUDA, instalar TensorRT, tener bien los drivers de NVIDIA porque los que vienen por default en Ubuntu son basura y sobretodo, que funcione la cosa. Podran notar que elimine muchisimos archivos anteriores de mis pruebas, esto debido a que mi logica de como hacer las cosas no funcionaba muy bien en la practica, no compilaba bien las builds usando ```colcon build``` y sobre todo la pelea fue mas con TensorRT.

**Nota para Lalo**:
No se como diablos lograste instalar el ```.deb``` porque en el repo del usv lo tienes en tu CMake de esta forma:

```{cmake}
set(TensorRT_INCLUDE_DIRS /usr/include/x86_64-linux-gnu)
set(TensorRT_INCLUDE_DIRS /usr/lib/x86_64-linux-gnu)
```

Y al menos a mi cuando intentaba instalarlo usando las guias de NVIDIA no me funcionaba, por lo que tuve que optar por usar la version del TAR file, que """en teoria""" me facilita muchisimo a la hora de especificar las rutas y al final me queda algo asi en mi CMake:

```{cmake}
if (NOT TensorRT_DIR)
    set(TensorRT_DIR /home/rob/libs/TensorRT-8.6.1.6/)
endif()
```

**Fin de la nota**

## ¿Como funciona?

Como mencione, una gran parte de las cosas elimine por lo que es una gran oportunidad de explicar como funciona todo de manera mas detallada.

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

## Ejecutar nodos

Abre tres terminales y corre lo siguiente:

Terminal 1:

```
colcon build
source install/setup.bash
ros2 run yolov8 ros_segmentation --ros-args -p model_path:=/home/rob/Downloads/yolov8n-pose.onnx -p camera_topic:=/camera/image_raw
```

Terminal 2:

```
source install/setup.bash
ros2 run camera_publisher camera_node
```

Terminal 3
```
rqt
```

Y fin. 





