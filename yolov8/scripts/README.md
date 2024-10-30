# Instala OpenCV con CUDA sin quedarte pelon en el intento

Asi es, para aquellos que no se han quedado pelones por el hecho de intentar instalar OpenCV con CUDA, les hice un script bien bonito que automatiza este proceso.

## Puntos a considerar

- Voy a asumir que ya instalaron CUDA usando el runfile.
- De igual manera, voy asumir tambien que tambien tienen instalado cudNN.

## Instalacion

Solo corran el siguiente comando si estan en raiz:

``` ./scripts/install_opencv.sh <OpenCV_Version> <CUDA_ARCH_BIN>```

Este utlimo argumento es la arquitectura de tu GPU de NVIDIA y varia por tipo de tarjeta/Jetson. Si no sabes cual es checa en la pagina oficial: https://developer.nvidia.com/cuda-gpus y si no aparece tu tarjeta grafica listada en teoria deberia de ser capaz de funcionar con CUDA, ya que todas las graficas que salieron a partir del 2008, son compatibles con CUDA, solo que tendras que hacer una investigacion en los forums.

