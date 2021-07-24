#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>


#include <iostream>
#include <stdio.h>

// tau = 2*pi. Un tau es una rotacion en radianes.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  
  //Inicializa el nodo de ros

  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  
  // Setup
  
  // Se define el grupo con el cual se va a planear, en este caso el manipulador.
  
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // Se usa la clase :planning_interface:`PlanningSceneInterface` para agregar o remover objectos y colisiones.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Configuraciones adicionales para mejorar la planeacion (Creo).
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



  // Se define un segundo grupo para controlar el gripper.


  static const std::string PLANNING_GROUP2 = "gripper";

  
  moveit::planning_interface::MoveGroupInterface move_group_interface_2(PLANNING_GROUP2);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_2;

  const moveit::core::JointModelGroup* joint_model_group_2 =
      move_group_interface_2.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);



  // Herramientas visuales, imprimir informacion.
  
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Start 

 // Se definen constantes de posicion y disatancia

 int pos;
 double pX;
 double pY;
 double pZ;
 double d;
 double c;
 double g;

 pX=0.262;
 pY=0.002;
 pZ=0.273;
 d=0.082;
 c=0.082;
 g=0;
 pos=0;

// Se inicia elcodigo esperando conmandos para ejecutar planes
 while(pos != 14)
{

// Comandos

printf("\n\n Selecciona una acción \n\n Tocar Boton\n\n [8] [4] [6] \n [3] [1] [2] \n [9] [5] [7] \n\n Otras opciones\n\n 10. Cerrar Gripper \n 11. Abrir Gripper \n 12. Empujar\n 13. Halar\n 14. Terminar\n");
   setbuf(stdin,NULL);
   scanf("%d",&pos);


  if (pos==1)
{
  
  // Define la posicion objectivo con respecto al ultimo link "camera_ur_mount".
  geometry_msgs::Pose target_pose1;
  
  target_pose1.orientation.x = 0.0;
  
  target_pose1.position.x = pX;
  target_pose1.position.y = pY;
  target_pose1.position.z = pZ;
  move_group_interface.setPoseTarget(target_pose1);
  
  

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();


}




 if (pos==2)
{  
  
  // Define la posicion objectivo con respecto al ultimo link "camera_ur_mount".

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.position.x = pX;
  target_pose1.position.y = pY-d;
  target_pose1.position.z = pZ;
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();
}

if (pos==3)
{  
  
  // Define la posicion objectivo con respecto al ultimo link "camera_ur_mount".

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.position.x = pX;
  target_pose1.position.y = pY+d;
  target_pose1.position.z = pZ;
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();
}

if (pos==4)
{  
  // Define la posicion objectivo con respecto al ultimo link "camera_ur_mount".
  

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.position.x = pX;
  target_pose1.position.y = pY;
  target_pose1.position.z = pZ+c;
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();
}

if (pos==5)
{  
  
  // Define la posicion objectivo con respecto al ultimo link "camera_ur_mount".

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.position.x = pX;
  target_pose1.position.y = pY;
  target_pose1.position.z = pZ-c;
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();
}
if (pos==6)
{  
  
  // Define la posicion objectivo con respecto al ultimo link "camera_ur_mount".

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.position.x = pX;
  target_pose1.position.y = pY-d;
  target_pose1.position.z = pZ+c;

  
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();
}
if (pos==7)
{  
  
  // Define la posicion objectivo con respecto al ultimo link "camera_ur_mount".

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.position.x = pX;
  target_pose1.position.y = pY-d;
  target_pose1.position.z = pZ-c;
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();
}
if (pos==8)
{  
  
  // Define la posicion objectivo con respecto al ultimo link "camera_ur_mount".

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.position.x = pX;
  target_pose1.position.y = pY+d;
  target_pose1.position.z = pZ+c;
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();
}
if (pos==9)
{  
  
  // Define la posicion objectivo con respecto al ultimo link "camera_ur_mount".

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.position.x = pX;
  target_pose1.position.y = pY+d;
  target_pose1.position.z = pZ-c;
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();
}


if (pos==15)
{  
  //Cambia la posicion de la muñeca unicamente. 
  
  // Crea un vector con las posiciones de cada joint del grupo "manipulator"
  std::vector<double> joints;
  joints = move_group_interface.getCurrentJointValues();
  // Define una nueva posicion para el joint 5
  joints.at(5)=tau/2;
  move_group_interface.setJointValueTarget(joints);
  move_group_interface.move();
  
}
if (pos==10)
{
  // Crea un vector con las posiciones de cada joint del grupo "gripper"
 
  std::vector<double> joints;
  joints = move_group_interface_2.getCurrentJointValues();
   
  // Define una nueva posicion para el joint 0 (gripper)
  joints.at(0)=80 * 3.1416 / 180;
  move_group_interface_2.setJointValueTarget(joints);
  move_group_interface_2.move();
}



if (pos==11)
{
 
  // Crea un vector con las posiciones de cada joint del grupo "gripper"

  std::vector<double> joints;
  joints = move_group_interface_2.getCurrentJointValues();
  
  // Define una nueva posicion para el joint 0 (gripper)
  joints.at(0)=0;
  move_group_interface_2.setJointValueTarget(joints);
  move_group_interface_2.move();
}


if (pos==16)
{
  // Cambia la orientación del ultimo link "camera_ur_mount".

  tf2::Quaternion q; 
  
// define la orientacion en grados de y,z,x
  q.setRPY(tau/4, 0.0 , -tau/20);
  q = q.normalize();
  

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = q[0];
  target_pose1.orientation.y = q[1];
  target_pose1.orientation.z = q[2]; 
  target_pose1.orientation.w = q[3]; 

  target_pose1.position.x = 0.197;
  target_pose1.position.y = -0.194;
  target_pose1.position.z = 0.208;
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();
}
if (pos==17)
{
// empuja en un angulo de 18
  geometry_msgs::PoseStamped actual;
  actual=move_group_interface.getCurrentPose();
  tf2::Quaternion q; 
  

  q.setRPY(tau/4, 0.0 , -tau/20);
  q = q.normalize();
  

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = q[0];
  target_pose1.orientation.y = q[1];
  target_pose1.orientation.z = q[2]; 
  target_pose1.orientation.w = q[3]; 

  target_pose1.position.x = actual.pose.position.x+0.003;
  target_pose1.position.y = actual.pose.position.y;
  target_pose1.position.z = actual.pose.position.z+0.009;
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();
}

if (pos==12)
{ 

  // empujar en x


  
  geometry_msgs::PoseStamped actual;
  actual=move_group_interface.getCurrentPose();
  
  geometry_msgs::Pose target_pose1;
  
  target_pose1.orientation.x = 0.0;
  
  target_pose1.position.x = actual.pose.position.x+0.01;
  target_pose1.position.y = actual.pose.position.y;
  target_pose1.position.z = actual.pose.position.z;
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();

}
if (pos==13)
{ 

  // Halar en x


  
  geometry_msgs::PoseStamped actual;
  actual=move_group_interface.getCurrentPose();
  
  geometry_msgs::Pose target_pose1;
  
  target_pose1.orientation.x = 0.0;
  
  target_pose1.position.x = actual.pose.position.x-0.01;
  target_pose1.position.y = actual.pose.position.y;
  target_pose1.position.z = actual.pose.position.z;
  move_group_interface.setPoseTarget(target_pose1);

  // Crea un nuevo plan para la trayectoria.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Planea a la posicion objectivo.

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Informacion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // Ejecuta el plan.
  move_group_interface.move();

}



}
   
  ros::shutdown();
  return 0;
}
