source /opt/ros/melodic/setup.bash
cd /home/robocol_folder/ERC_Arm_2021
source devel/setup.bash
roslaunch Kinematics objective1.launch
rosrun Kinematics exe_task_2
rosrun Kinematics exe_task_3