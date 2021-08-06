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

//Codigo para crear el topico
void chatterCallback(const std_msgs::String::ConstPtr& msg)
  {
     ROS_INFO("I heard: [%s]", msg->data.c_str());
  }

void Inicial_Pose(
moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  //Se mueve a la posicion inicial. 
  
  // Crea un vector con las posiciones de cada joint del grupo "manipulator"
  std::vector<double> joints;
  joints = move_group_interface.getCurrentJointValues();
  // Define una nueva posicion para los joints
  joints.at(0)= 0;
  joints.at(1)=-tau/3;
  joints.at(2)= tau*0.277; 
  joints.at(3)= tau/18;
  joints.at(4)= tau/4;
  joints.at(5)= -tau/4;
  move_group_interface.setJointValueTarget(joints);
  move_group_interface.move();


}


void close_gripper(
moveit::planning_interface::MoveGroupInterface &move_group_interface_2)
{
  std::vector<double> joints;
  joints = move_group_interface_2.getCurrentJointValues();
   
  // Define una nueva posicion para el joint 0 (gripper)
  joints.at(0)=80 * tau / 360;
  move_group_interface_2.setJointValueTarget(joints);
  move_group_interface_2.move();


}

void open_gripper(
moveit::planning_interface::MoveGroupInterface &move_group_interface_2)
{
  std::vector<double> joints;
  joints = move_group_interface_2.getCurrentJointValues();
   
  // Define una nueva posicion para el joint 0 (gripper)
  joints.at(0)=0;
  move_group_interface_2.setJointValueTarget(joints);
  move_group_interface_2.move();


}

void go_close(
moveit::planning_interface::MoveGroupInterface &move_group_interface)
{

 tf2::Quaternion q; 
  
// define la orientacion en grados de y,z,x
  q.setRPY(tau/4, 0 , 0);
  q = q.normalize();
  

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = q[0];
  target_pose1.orientation.y = q[1];
  target_pose1.orientation.z = q[2]; 
  target_pose1.orientation.w = q[3]; 

  target_pose1.position.x = 0.260;
  target_pose1.position.y = 0.084;
  target_pose1.position.z = 0.417;
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


void right(
moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
//mover a la derecha. 
  
  tf2::Quaternion q; 
  
// define la orientacion en grados de y,z,x
  q.setRPY(tau/4, 0 , 0);
  q = q.normalize();
  

  
  
  geometry_msgs::PoseStamped actual;
  actual=move_group_interface.getCurrentPose();
  
  geometry_msgs::Pose target_pose1;
  
  target_pose1.orientation.x = q[0];
  target_pose1.orientation.y = q[1];
  target_pose1.orientation.z = q[2]; 
  target_pose1.orientation.w = q[3]; 
  
  target_pose1.position.x = actual.pose.position.x;
  target_pose1.position.y = actual.pose.position.y-0.008;
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

void left(
moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
//mover a la izuiqerda
  
  tf2::Quaternion q; 
  
// define la orientacion en grados de y,z,x
  q.setRPY(tau/4, 0 , 0);
  q = q.normalize();
  

  
  
  geometry_msgs::PoseStamped actual;
  actual=move_group_interface.getCurrentPose();
  
  geometry_msgs::Pose target_pose1;
  
  target_pose1.orientation.x = q[0];
  target_pose1.orientation.y = q[1];
  target_pose1.orientation.z = q[2]; 
  target_pose1.orientation.w = q[3]; 
  
  target_pose1.position.x = actual.pose.position.x;
  target_pose1.position.y = actual.pose.position.y+0.008;
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


void down(
moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
//mover a la izuiqerda
  
  tf2::Quaternion q; 
  
// define la orientacion en grados de y,z,x
 q.setRPY(tau/4, 0 , 0);
  q = q.normalize();
  

  
  
  geometry_msgs::PoseStamped actual;
  actual=move_group_interface.getCurrentPose();
  
  geometry_msgs::Pose target_pose1;
  
  target_pose1.orientation.x = q[0];
  target_pose1.orientation.y = q[1];
  target_pose1.orientation.z = q[2]; 
  target_pose1.orientation.w = q[3]; 
  
  target_pose1.position.x = actual.pose.position.x;
  target_pose1.position.y = actual.pose.position.y;
  target_pose1.position.z = actual.pose.position.z-0.008;
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

void up(
moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
//mover a la izuiqerda
  
  tf2::Quaternion q; 
  
// define la orientacion en grados de y,z,x
  q.setRPY(tau/4, 0 , 0);
  q = q.normalize();
  

 
  geometry_msgs::PoseStamped actual;
  actual=move_group_interface.getCurrentPose();
  
  geometry_msgs::Pose target_pose1;
  
  target_pose1.orientation.x = q[0];
  target_pose1.orientation.y = q[1];
  target_pose1.orientation.z = q[2]; 
  target_pose1.orientation.w = q[3]; 
  
  target_pose1.position.x = actual.pose.position.x;
  target_pose1.position.y = actual.pose.position.y;
  target_pose1.position.z = actual.pose.position.z+0.008;
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





int main(int argc, char** argv)
{
  
  //Inicializa el nodo de ros

  ros::init(argc, argv, "Task_1_Topic");
  ros::NodeHandle node_handle;
   
  ros::Subscriber sub = node_handle.subscribe("chatter", 1000, chatterCallback);

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

  pX=0.260;
  pY=0.084;
  pZ=0.417;
  d=0.08;
  c=0.15;
  pos=0;
   
 
  Inicial_Pose(move_group_interface);
  close_gripper(move_group_interface_2);
  go_close(move_group_interface);
  
  while(pos!=9)
  {
   printf("\n\n Selecciona una acción \n\n 1. Arriba \n 2. Abajo \n 3. Derecha \n 4. Izquierda \n 9. Salir\n");
   setbuf(stdin,NULL);
   scanf("%d",&pos);
  if(pos==1)
  {
   up(move_group_interface);
  }
  if(pos==2)
  {
   down(move_group_interface);
  }

  if(pos==3)
  {
   right(move_group_interface);
  }

  if(pos==4)
  {
   left(move_group_interface);
  }
  
  


   
  }

  open_gripper(move_group_interface_2);
    
 
    



  ros::shutdown();
  return 0;
}



