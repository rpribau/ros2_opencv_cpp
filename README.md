# ros2_opencv_cpp

**Este repo son meras pruebas que voy haciendo de tiempo en tiempo para el SDV para [@Vanttec](https://github.com/vanttec). El codigo no va ser el mismo que tenia hace uno o dos meses, por lo que si quieres ver versiones anteriores buscalas por los commits.**


## Updates

25/10/2024

Ha pasado un ratote eh... pues si, realmente ha pasado un ratote y en ese ratote volvi a hacer todo lo que tenia anteriormente pero ahora con OpenCV con CUDA y TensorRT. Logre correr unas cosas que venian en el repo de usv e intentando basarme lo mas que pueda. Ademas de que tuve que rehacer el nodo de Frida, el cual lo deje en Python porque la neta mi pensamiento fue el siguiente:

- Nodo funcionar bien, camara enciende bien uga ugh, editar codigo para quitar la logica de YOLO para que pueda ser usado en otros lados y listo.

**NOTA**, falta agregar una libreria que estoy terminando de desarrollar con TensorRT que me permita correrlo con la logica que tenia Frida de las distancias del objeto con respecto a la camara.

14/06/2024

Esta prueba es el uso de dos nodos de ROS2, uno que se encarga de correr la camara con un publisher **image_raw** y el otro nodo solo tiene un subscriber que toma el video para correr el modelo ```yolov8-pose.onnx``` en otro nodo por aparte.

INTENTE muchas formas de correrlo directamente desde el primer nodo, solo que al correr me marcaba constantemente errores de memoria (basicamente decia que necesitaba casi 10 TB de memoria lol). 

Como use un codigo ya hecho con un nodo para la camara de mi lap y no me deja subir la carpeta aqui dejo el recurso que use: https://gitlab.com/boldhearts/ros2_v4l2_camera

**Cosas a considerar a un futuro**

Para la Multisense va tocar buscar o hacer un codigo exactamente igual que el de ```v4l2_camera``` y luego al final agregar el resto de la logica de las distancias (algo que proximamente trabajare).

------------------------------------------------------------------------------------------------------------------

22/05/2024

Asi es, este el el repo de las pruebas no oficiales de vision para Vanttec. Si lo tengo por fuera y PUBLICO es por la mera comodidad. Este primer codigo como tal no funciona, tiene un error de memoria muy duro.
