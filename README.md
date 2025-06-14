# ros2_opencv_cpp

**Este repo son meras pruebas que voy haciendo de tiempo en tiempo para el SDV para [@Vanttec](https://github.com/vanttec). El codigo no va ser el mismo que tenia hace uno o dos meses, por lo que si quieres ver versiones anteriores buscalas por los commits.**

TO-DO

- [x] Crear el launch file
- [x] Crear test con camara de laptop
- [x] Editar el launch file para que inicialice tambien la multisense



**Nota del momento**: 

light work no reaction...

<p align="center">
  <img src="https://preview.redd.it/keep-rollin-rollin-rollin-rollin-what-v0-08zawu3p8mae1.jpeg?auto=webp&s=5becad1591079dc7f3688934ffd5e36d74ecc08f" alt="Imagen centrada" width="600"/>
</p>


## Introduccion

Decian que mi 6to semestre en mi carrera de IDM seria el caos, que me volveria calvo por analisis topologico y en general perderia mucha vida social... no se equivocaron. Sin embargo, eso no significa que no pueda trabajar en esta cosa en los intervalos de tiempo. Estos son los nuevos cambios que hice:

- Actualice un poco el codigo en general, que fuese un poco mas optimo para que no explote la Orinbb

- FINALMENTE HAY UN LAUNCH FILE en ```/launch/detection_launch.py``` que automatiza todo el proceso de ejecucion de codigo y ya no tienes que abrir 3 terminales para el topic de la camara, la deteccion y rqt. Ahora solo son 2!

## Requerimientos

Voy a asumir que ya esta lo siguiente instalado por JetPack:

- Ubuntu 22.04
- ROS2 Humble
- CUDA >= 12.2
- cuDNN >= 9
- TensorRT >= 10
- gcc = 11.4
- g++ = 11.4


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
ros2 launch yolov8_tensorrt_cpp detection.launch.py model_type:=object model_path:=/ruta/absoluta/a/tu/yolov8n.onnx
```

- Terminal 2
```
rqt
```

La forma en como funciona se basa principalmente en la <a href="https://github.com/cyrusbehr/tensorrt-cpp-api">TensorRT C++ API de cyrusbehr</a> que la verdad es la cosa mas bonita que haya visto en mi vida. Lo que hace toda la parte de ```libs/tensorrt-cpp-api``` se encarga de crear el modelo de ONNX a un ```.engine``` para poder generar los modelos mas facilmente. 


