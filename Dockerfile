FROM  ghcr.io/europeanroverchallenge/erc-remote-image-base:latest

SHELL ["/bin/bash","-c"]

WORKDIR /home/robocol_folder
RUN sudo apt-get update
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update
ADD https://api.github.com/repos/robocol-rem-u/ERC_Arm_2021/git/refs/heads/develop version.json
RUN git clone -b develop https://github.com/robocol-rem-u/ERC_Arm_2021.git
RUN rosdep update
RUN rosdep install --from-paths /home/robocol_folder/ERC_Arm_2021/src/ --ignore-src --rosdistro melodic -r -y
RUN sudo apt install ros-melodic-teleop* -y
RUN sudo apt install ros-melodic-joy* -y
RUN sudo apt install ros-melodic-aruco-ros* -y
RUN sudo apt-get install ros-melodic-ros-controllers* -y
RUN sudo apt install -y python-pip
RUN sudo apt install -y python3-pip
RUN python3 -m pip uninstall opencv-python
RUN python3 -m pip install opencv-contrib-python
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /home/robocol_folder/ERC_Arm_2021; catkin build'
RUN echo ". /opt/ros/melodic/setup.bash" >> /etc/bash.bashrc
RUN echo ". /home/robocol_folder/ERC_Arm_2021/devel/setup.bash" >> /etc/bash.bashrc
RUN /bin/bash -c '. /home/robocol_folder/ERC_Arm_2021/devel/setup.bash'