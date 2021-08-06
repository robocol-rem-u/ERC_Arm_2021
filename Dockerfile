FROM  ghcr.io/europeanroverchallenge/erc-remote-image-base

SHELL ["/bin/bash","-c"]

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y lsb-core g++ openssh-server gedit vim

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update

RUN apt-get update && apt-get install -y gdb gnupg2 apt-transport-https



RUN wget -O /tmp/gazebo5_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo5_install.sh; sudo sh /tmp/gazebo5_install.sh

RUN apt install ros-melodic-industrial-core -y
#teleop
RUN apt install ros-melodic-teleop* -y

#joystick
RUN apt install ros-melodic-joy* -y

#aruco
RUN apt install ros-melodic-aruco-ros* -y

#controller
RUN apt install ros-melodic-ros-controllers* -y

#pip2
RUN apt install -y python-pip

#pip3
RUN apt install -y python3-pip


RUN sudo apt install -y ros-melodic-moveit
RUN sudo apt-get install -y ros-melodic-gazebo-ros
RUN sudo apt-get install -y ros-melodic-control-toolbox
RUN sudo apt install -y ros-melodic-tf-conversions
RUN sudo apt-get install -y ros-melodic-roslint
RUN sudo apt install -y ros-melodic-graph-msgs
RUN sudo apt-get install -y ros-melodic-moveit-visual-tools

RUN source /opt/ros/melodic/setup.bash \

RUN git clone https://github.com/robocol-rem-u/ERC_Arm_2021.git \ 
    && cd ERC_Arm_2021 \ 
    && git checkout develop

RUN cd ~/ERC_Arm_2021
RUN catkin_make

RUN source devel/setup.bash


CMD ["roslaunch", "simulation", "simulator.launch"]  