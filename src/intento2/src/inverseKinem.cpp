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


// get data console  

#include <iostream>
#include <stdio.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  

  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  
  // Setup
  
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "manipulator";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Start 
  
  

  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.


 int pos;
 double pX;
 double pY;
 double pZ;
 double d;
 double c;
 double g;

 //printf("Selecciona una accion \n 0. Iniciar \n 9. Terminar ");
 //setbuf(stdin,NULL);
 //scanf("%d",&pos);
 pX=0.262;
 pY=0.002;
 pZ=0.273;
 d=0.082;
 c=0.082;
 g=0;
 pos=0;
 while(pos != 12)
{
// read data pose
 
   
   printf("\n\n Selecciona una acción \n\n Tocar Boton\n\n [8] [4] [6] \n [3] [1] [2] \n [9] [5] [7] \n\n Otras opciones\n\n 10. Cerrar Gripper \n 11. Abrir Gripper \n 12. Terminar\n");
   setbuf(stdin,NULL);
   scanf("%d",&pos);
   //printf("Intentando Pose:   %d",pos);

  if (pos==1)
{
  

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 1.0;
  target_pose1.orientation.y = 1.0;
  target_pose1.orientation.z = 1.0;
  target_pose1.orientation.w = 2.0;
  target_pose1.position.x = pX;
  target_pose1.position.y = pY;
  target_pose1.position.z = pz;
  move_group_interface.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group_interface.move();
}




 if (pos==2)
{  
  
  

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = 1.0;
  target_pose2.orientation.y = 1.0;
  target_pose2.orientation.z = 1.0;
  target_pose2.orientation.w = 2.0;
  target_pose2.position.x = pX;
  target_pose2.position.y = pY-d;
  target_pose2.position.z = pZ;
  move_group_interface.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group_interface.move();
}

if (pos==3)
{  
  
  

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = 1.0;
  target_pose2.orientation.y = 1.0;
  target_pose2.orientation.z = 1.0;
  target_pose2.orientation.w = 2.0;
  target_pose2.position.x = pX;
  target_pose2.position.y = pY+d;
  target_pose2.position.z = pZ;
  move_group_interface.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group_interface.move();
}

if (pos==4)
{  
  
  

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = 1.0;
  target_pose2.orientation.y = 1.0;
  target_pose2.orientation.z = 1.0;
  target_pose2.orientation.w = 2.0;
  target_pose2.position.x = pX;
  target_pose2.position.y = pY;
  target_pose2.position.z = pZ+c;
  move_group_interface.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group_interface.move();
}

if (pos==5)
{  
  
  

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = 1.0;
  target_pose2.orientation.y = 1.0;
  target_pose2.orientation.z = 1.0;
  target_pose2.orientation.w = 2.0;
  target_pose2.position.x = pX;
  target_pose2.position.y = pY;
  target_pose2.position.z = pZ-c;
  move_group_interface.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group_interface.move();
}
if (pos==6)
{  
  
  

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = 1.0;
  target_pose2.orientation.y = 1.0;
  target_pose2.orientation.z = 1.0;
  target_pose2.orientation.w = 2.0;
  target_pose2.position.x = pX;
  target_pose2.position.y = pY-d;
  target_pose2.position.z = pZ+c;
  move_group_interface.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group_interface.move();
}
if (pos==7)
{  
  
  

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = 1.0;
  target_pose2.orientation.y = 1.0;
  target_pose2.orientation.z = 1.0;
  target_pose2.orientation.w = 2.0;
  target_pose2.position.x = pX;
  target_pose2.position.y = pY-d;
  target_pose2.position.z = pZ-c;
  move_group_interface.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group_interface.move();
}
if (pos==8)
{  
  
  

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = 1.0;
  target_pose2.orientation.y = 1.0;
  target_pose2.orientation.z = 1.0;
  target_pose2.orientation.w = 2.0;
  target_pose2.position.x = pX;
  target_pose2.position.y = pY+d;
  target_pose2.position.z = pZ+c;
  move_group_interface.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group_interface.move();
}
if (pos==9)
{  
  
  

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = 1.0;
  target_pose2.orientation.y = 1.0;
  target_pose2.orientation.z = 1.0;
  target_pose2.orientation.w = 2.0;
  target_pose2.position.x = pX;
  target_pose2.position.y = pY+d;
  target_pose2.position.z = pZ-c;
  move_group_interface.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group_interface.move();
}


if (pos==15)
{  
  //g=g-0.2;
 
  std::vector<double> joints;
  joints = move_group_interface.getCurrentJointValues();
  //printf(joints);
  joints.at(5)=tau/2;
  move_group_interface.setJointValueTarget(joints);
  move_group_interface.move();
  
}
if (pos==10)
{
 
  

  // ros::init(argc, argv, "gripp");
  //ros::NodeHandle n;
  
 ros::Publisher pub = node_handle.advertise<std_msgs::String>("/gripper_command", 1000);
 int con;
 while (ros::ok())
{
  
  std_msgs::String msg;
  msg.data = "close";
  pub.publish(msg);

  printf("\n 1. Continuar\n 9. Menu principal\n");
  setbuf(stdin,NULL);
  scanf("%d",&con);
  if (con==9)
  {
   break;
  }
}
}


if (pos==11)
{
 
  

  // ros::init(argc, argv, "gripp");
  //ros::NodeHandle n;
  
 ros::Publisher pub = node_handle.advertise<std_msgs::String>("/gripper_command", 1000);
 int con;
 while (ros::ok())
{
  
  std_msgs::String msg;
  msg.data = "open";
  pub.publish(msg);

  printf("\n 1. Continuar\n 9. Menu principal\n");
  setbuf(stdin,NULL);
  scanf("%d",&con);
  if (con==9)
  {
  break;
  }
}
}
 
  
}
   
  ros::shutdown();
  return 0;
}
