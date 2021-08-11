# ERC_Arm_2021
Para usar esta simulación se requiere tener instalado:
- Ubuntu 18.04:
La instalación de Ubuntu 18.04 se encuentra en el siguiente link:
https://ubuntu.com/download/desktop
- ROS Melodic. 
La instalación de ROS Melodic para Ubuntu 18.04 se encuentra en el siguiente link: 
http://wiki.ros.org/melodic/Installation/Ubuntu
- Moveit. 
La instalación de Moveit se encuentra en el siguiente link: (Se requiere tener ROS para instalar MoveIt) 
http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html

## Descargar e installar simulación

Para descargar e instalar la simulacion ejecutar los siguientes comandos:


```console
sudo apt-get update
rosdep init && rosdep update
git clone -b develop https://github.com/robocol-rem-u/ERC_Arm_2021.git
rosdep update
rosdep install --from-paths ~/ERC_Arm_2021/src/ --ignore-src --rosdistro melodic -r -y
sudo apt install ros-melodic-teleop* -y
sudo apt install ros-melodic-joy* -y
sudo apt install ros-melodic-aruco-ros* -y
sudo apt-get install ros-melodic-ros-controllers* -y
sudo apt install -y python-pip
sudo apt install -y python3-pip
```
NOTA IMPORTANTE: ANTES DE CONTINUAR CON LOS SIGUIENTES COMANDOS Y PARA EVITAR ERRORES EN LA SIMULACION O EL DOCKER ELIMINAR LA CARPETA "ERC_Arm_2021/build" y "ERC_Arm_2021/devel".  

```console
cd ~/ERC_Arm_2021
catkin_make
source /opt/ros/melodic/setup.bash
```
## Launch files

### Ejecutar simulación


Para ejecutar la simulacion usar:

```console
source devel/setup.bash
roslaunch simulation simulator.launch
```

### Ejecutables

para ejecutar otro nodo es necesario abrir un nuevo terminal
Para correr cada una de las tareas ejecutar los siguientes comandos :

 ```console
cd /ERC_Arm_2021
source devel/setup.bash
rosrun Kinematics exe_task_1
rosrun Kinematics exe_task_2
rosrun Kinematics exe_task_3
```
Cada rosrun ejecuta una tarea.

## calibracion_vision_pkg (paquete de visión)
Es el workspace donde está guardado el trabajo de visión para el ERC 2021 Maintenance Task

Contiene un solo paquete llamado __calibracion_vision_pkg__

### Dependencias
+ cv_bridge
+ roscpp
+ rospy
+ std_msgs

Para asegurarse de que cv_bridge esté instalado y no genere errores:
```
sudo apt install ros-melodic-cv-bridge
```
### Librerías necesarias de python

Adicionalmente, para que el código funcione de la manera esperada se necesita la librería opencv-contrib, la cual puede instalarse utilizando los siguientes comandos en la terminal:
```
python3 -m pip uninstall opencv-python
python3 -m pip install opencv-contrib-python
```
(Cabe aclarar que el primero de los comandos es obligatorio para evitar conflictos en la librería que impidan su correcta ejecución)

### Correr el script

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

## ¿Cómo utilizar el docker?

### 1. Tener la imagen

Para utilizar el docker, primero debe tener Docker Engine instalada en su computador. Para esto, consultar la documentación de Docker
- https://docs.docker.com/engine/install/

Una vez instalada, tener la imagen de docker (Docker Image) disponible en su computador. Para esto hay dos opciones:

1. Clonar la imagen de docker directamente desde el repositorio creado para Maintenance Task en Docker Hub. Para realizarlo, ejecute el siguiente comando:
```
docker clone robocol/maintenance_2021:latest
```
Debe verificar que en la imagen que está en el repositorio se encuentran los últimos cambios realizados al proyecto

2. 

### Ejecutar la imagen

# TO-DO

## Visión

1. Verificar los tópicos en el último enviroment Update
2. Investigar acerca del camera_info y cómo se puede usar para mapear el ambiente
3. Distribuir al equipo en las tareas

## Brazo

## Interfaz




