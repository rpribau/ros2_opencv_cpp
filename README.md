# ros2_opencv_cpp

**Este repo son meras pruebas que voy haciendo de tiempo en tiempo para el SDV para [@Vanttec](https://github.com/vanttec). El codigo no puede ser el mismo que tenia hace un o dos meses, por lo que si quieres ver versiones anteriores buscalas por los commits.**


## Updates

14/06/2024

Esta prueba es el uso de dos nodos de ROS2, uno que se encarga de correr la camara con un publisher **image_raw** y el otro nodo solo tiene un subscriber que toma el video para correr el modelo ```yolov8-pose.onnx``` en otro nodo por aparte.

INTENTE muchas formas de correrlo directamente desde el primer nodo, solo que al correr me marcaba constantemente errores de memoria (basicamente decia que necesitaba casi 10 TB de memoria lol). 

Como use un codigo ya hecho con un nodo para la camara de mi lap y no me deja subir la carpeta aqui dejo el recirso que use: https://gitlab.com/boldhearts/ros2_v4l2_camera

**Cosas a considerar a un futuro**

Para la Multisense va tocar buscar o hacer un codigo exactamente igual que el de ```v4l2_camera``` y luego al final agregar el resto de la logica de las distancias (algo que proximamente trabajare).

------------------------------------------------------------------------------------------------------------------

22/05/2024

Asi es, este el el repo de las pruebas no oficiales de vision para Vanttec. Si lo tengo por fuera y PUBLICO es por la mera comodidad. Este primer codigo como tal no funciona, tiene un error de memoria muy duro.
