#ifndef YOLOV8_MAIN_H
#define YOLOV8_MAIN_H

#include <opencv2/opencv.hpp> // Incluye las bibliotecas de OpenCV
#include "YoloPose.h"      // Ajusta según el nombre y ubicación de tus archivos de YOLO
#include "ImageTools.h"    // Ajusta según el nombre y ubicación de tus herramientas de procesamiento de imágenes

void yoloPose(cv::Mat &image, YoloPose &yolo);

#endif // YOLOV8_MAIN_H

