# calibracion_vision_ws
Es el workspace donde está guardado el trabajo de visión para el ERC 2021 Maintenance Task

Contiene un solo paquete llamado __calibracion_vision_pkg__

## Dependencias
+ cv_bridge
+ roscpp
+ rospy
+ std_msgs

Para asegurarse de que cv_bridge esté instalado y no genere errores:
```
sudo apt install ros-melodic-cv-bridge
```
## Librerías necesarias de python

Adicionalmente, para que el código funcione de la manera esperada se necesita la librería opencv-contrib, la cual puede instalarse utilizando los siguientes comandos en la terminal:
```
python3 -m pip uninstall opencv-python
python3 -m pip install opencv-contrib-python
```
(Cabe aclarar que el primero de los comandos es obligatorio para evitar conflictos en la librería que impidan su correcta ejecución)

## Correr el script

Para correr el script se deben seguir los siguientes 3 pasos:
1. Veificar que exista un tópico que publique una imagen de tipo _Image_
2. En __calibracion_vision_node.py__ la variable __IMAGE_TOPIC__ debe modificarse para emparejar el tópico que envía la imagen (Por default es el tópico _/camera_raw_ de la simulación ERC_2021_Maintenance_Task
3. Correr los siguientes comandos en una terminal:
```
cd calibracion_vision_ws
source devel/setup.bash
cd src/calibracion_vision_pkg/scripts
chmod +x calibracion_vision_node.py
cd ../../..
rosrun calibracion_vision_pkg calibracion_vision_node.py
```
Una vez ejecutado el último comando, aparecerá una ventana llamada _image_ que contendrá la imagen recibida por el tópico junto con el procesamiendo debido
