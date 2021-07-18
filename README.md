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
rosdep install --from-paths /ERC_Arm_2021/src/ --ignore-src --rosdistro melodic -r -y
sudo apt install ros-melodic-teleop* -y
sudo apt install ros-melodic-joy* -y
sudo apt install ros-melodic-aruco-ros* -y
sudo apt-get install ros-melodic-ros-controllers* -y
sudo apt install -y python-pip
sudo apt install -y python3-pip
cd /ERC_Arm_2021
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

### Manipular Simulacion mediante codigo

Para ejecutar el codigo y manipular la simulacion:

 Abrir un nuevo terminal y ejecutar los siguientes comandos para el codigo principal. 

 ```console
cd /ERC_Arm_2021
source devel/setup.bash
rosrun Kinematics task_1
```
Desde acá se puede  manipular tanto el brazo como el gripper.


