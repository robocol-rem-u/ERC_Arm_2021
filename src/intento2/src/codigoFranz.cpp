#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>	
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Grupo de planeación
static const std::string PLANNING_GROUP = "manipulator";
// Pose del texto
Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
// Vector de posiciones de los joints	
std::vector<double> joint_group_positions;
// Variable booleana para indicar que una operación fue exitosa.
bool success;
// Namespace
namespace rvt = rviz_visual_tools;

void planning_to_pose_goal(
	moveit::planning_interface::MoveGroupInterface &move_group,
	moveit_visual_tools::MoveItVisualTools &visual_tools,
	const moveit::core::JointModelGroup* joint_model_group,
	moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
	// Planeación a meta de pose (pose goal)
	// ^^^^^^^^^^^^^^^^^^^^^^^
	// Planea el movimiento del grupo a una pose deseada del end-effector.
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo\n");
	// Cuaternión a partir de ángulos de Euler.
	tf2::Quaternion q;
	geometry_msgs::Pose target_pose1;
	q.setRPY(0.0, 3.14, 0.0);
	q = q.normalize();
	printf(" Quaternion ->  x: %f y: %f z: %f w: %f\n", q[0], q[1], q[2], q[3]);
	// Pose objetivo
	target_pose1.orientation.x = q[0];
	target_pose1.orientation.y = q[1];
	target_pose1.orientation.z = q[2];
	target_pose1.orientation.w = q[3];
	target_pose1.position.x =  3.0;
	target_pose1.position.y =  0.0;
	target_pose1.position.z =  5.0;
	move_group.setPoseTarget(target_pose1);
	
	bool success = (move_group.plan(my_plan) ==         		   moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("robocol", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	// Visualiazación de planeación
	// ^^^^^^^^^^^^^^^^^
	// We can also visualize the plan as a line with markers in RViz.
	ROS_INFO_NAMED("robocol", "Visualizing plan 1 as trajectory line");
	visual_tools.publishAxisLabeled(target_pose1, "pose1");
	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo\n");
	// Moviendo a pose objetivo
	// ^^^^^^^^^^^^^^^^^^^^^
	// Descomentar la siguiente línea cuando se trabaja con un robot real.
	move_group.move();
}

void planning_to_joint_space_goal(
	moveit::planning_interface::MoveGroupInterface &move_group,
	moveit_visual_tools::MoveItVisualTools &visual_tools,
	const moveit::core::JointModelGroup* joint_model_group,
	moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
	moveit::core::RobotStatePtr &current_state)
{
	// Planeación a un objetivo de espacio de juntura
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Let's set a joint space goal and move towards it. This will replace the pose target we set above.
	// To start, we'll create an pointer that references the current robot's state.
	// RobotState is the object that contains all the current position/velocity/acceleration data.
	current_state = move_group.getCurrentState();
	// Next get the current set of joint values for the group.
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo 3\n");
	// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
	joint_group_positions[5] -= 0.8;  // radians
	move_group.setJointValueTarget(joint_group_positions);
	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("manipulator", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
	// Visualize the plan in RViz
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	// Moverse a la pose objetivo
	// ^^^^^^^^^^^^^^^^^^^^^
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo 4\n");
	move_group.move();
}

// void planning_cartesian_path(const moveit::planning_interface::MoveGroupInterface& move_group)//, )
void planning_cartesian_path(
	moveit::planning_interface::MoveGroupInterface &move_group,
	moveit_visual_tools::MoveItVisualTools &visual_tools)
{
	// Cartesian Paths
	// ^^^^^^^^^^^^^^^
	// Planeación cartersiana por medio de waypoints.
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo 5\n");
	geometry_msgs::PoseStamped start_pose = move_group.getCurrentPose();
	// Vector de waypoints
	std::vector<geometry_msgs::Pose> waypoints;
	// Agregar waypoint
	waypoints.push_back(start_pose.pose);
	// Iniciar pose de destino como posición inicial.
	geometry_msgs::Pose target_pose = start_pose.pose;
	target_pose.position.z -= 2.0;
	waypoints.push_back(target_pose);  // down
	// Los movimientos cartesianos se deben de hacer lento.
	move_group.setMaxVelocityScalingFactor(0.1);
	// We want the Cartesian path to be interpolated at a resolution of 1 cm
	// which is why we will specify 0.01 as the max step in Cartesian
	// translation.  We will specify the jump threshold as 0.0, effectively disabling it.
	// Warning - disabling the jump threshold while operating real hardware can cause
	// large unpredictable motions of redundant joints and could be a safety issue
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	ROS_INFO_NAMED("robocol", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
	// move_group.move();
	move_group.execute(trajectory);
	// Visualizar la planeación en RViz
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
	for (std::size_t i = 0; i < waypoints.size(); ++i)
		visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
	visual_tools.trigger();
}

void inverse_kinematics(moveit_visual_tools::MoveItVisualTools &visual_tools)
{
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo\n");
	printf("--- Cinemática ---\n");
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	// Modelo cinemático
	const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
	// Estado cinemático
	moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
	// Cinemática directa
	printf("  Cinemática directa:\n");
	kinematic_state->setToRandomPositions(joint_model_group);
	const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("wrist_3_link");
	ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
	ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
	// Cinemática inversa
	printf("  Cinemática inversa:\n");
	double timeout = 0.1;
	std::vector<double> joint_values;
	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);
	if (found_ik)
	{
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for (std::size_t i = 0; i < joint_names.size(); ++i)
		{
			ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		}
	}
	else
	{
		ROS_INFO("Did not find IK solution");
	}
	// Jacobiano
	Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
	Eigen::MatrixXd jacobian;
	kinematic_state->getJacobian(joint_model_group,	kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
	ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
}

void object_management(
	moveit::planning_interface::MoveGroupInterface &move_group,
	moveit_visual_tools::MoveItVisualTools &visual_tools,
	const moveit::core::JointModelGroup* joint_model_group,
	moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
	moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
	// Adding/Removing Objects and Attaching/Detaching Objects
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Define a collision object ROS message.
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo 6\n");
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = move_group.getPlanningFrame();
	// The id of the object is used to identify it.
	collision_object.id = "box1";
	// Define a box to add to the world.
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 1.50;
	primitive.dimensions[1] = 0.52;
	primitive.dimensions[2] = 1.50;
	// Define a pose for the box (specified relative to frame_id)
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = 3.0;
	box_pose.position.y = 0.0;
	box_pose.position.z = 1.0;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);
	// Now, let's add the collision object into the world
	ROS_INFO_NAMED("robocol", "Add an object into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);
	// Show text in RViz of status
	visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();
	// Wait for MoveGroup to recieve and process the collision object message
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz\n");
	// Cuaternión a partir de ángulos de Euler.
	tf2::Quaternion q;
	q.setRPY(0.0, 3.14, 0.8);
	q = q.normalize();
	printf(" Quaternion ->  x: %f y: %f z: %f w: %f\n", q[0], q[1], q[2], q[3]);
	// Now when we plan a trajectory it will avoid the obstacle
	move_group.setStartState(*move_group.getCurrentState());
	geometry_msgs::Pose another_pose;	
	// Pose objetivo
	another_pose.orientation.x = q[0];
	another_pose.orientation.y = q[1];
	another_pose.orientation.z = q[2];
	another_pose.orientation.w = q[3];
	another_pose.position.x =  3.0;
	another_pose.position.y =  0.0;
	another_pose.position.z =  2.5;
	move_group.setPoseTarget(another_pose);
	// Plan
	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("robocol", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");
	// Visualize the plan in RViz
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("next step\n");
	// Move
	move_group.move();
	visual_tools.prompt("next step\n");
	// Now, let's attach the collision object to the robot.
	ROS_INFO_NAMED("robocol", "Attach the object to the robot");
	move_group.attachObject(collision_object.id);
	// Show text in RViz of status
	visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();
	/* Wait for MoveGroup to recieve and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the robot\n");
	// Cuaternión a partir de ángulos de Euler.
	q.setRPY(0.0, 3.14, 3.14);
	q = q.normalize();
	printf(" Quaternion ->  x: %f y: %f z: %f w: %f\n", q[0], q[1], q[2], q[3]);
	// Now when we plan a trajectory it will avoid the obstacle
	move_group.setStartState(*move_group.getCurrentState());
	// Pose objetivo
	another_pose.orientation.x = q[0];
	another_pose.orientation.y = q[1];
	another_pose.orientation.z = q[2];
	another_pose.orientation.w = q[3];
	another_pose.position.x =  2.0;
	another_pose.position.y =  2.0;
	another_pose.position.z =  2.5;
	move_group.setPoseTarget(another_pose);
	// Plan
	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("robocol", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");
	// Visualize the plan in RViz
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("next step\n");
	// Move
	move_group.move();
	// Now, let's detach the collision object from the robot.
	ROS_INFO_NAMED("robocol", "Detach the object from the robot");
	move_group.detachObject(collision_object.id);
	// Show text in RViz of status
	visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();
	// Wait for MoveGroup to recieve and process the collision object message
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz\n");
	// Cuaternión a partir de ángulos de Euler.
	q.setRPY(0.0, 3.14, 3.14);
	q = q.normalize();
	printf(" Quaternion ->  x: %f y: %f z: %f w: %f\n", q[0], q[1], q[2], q[3]);
	// Now when we plan a trajectory it will avoid the obstacle
	move_group.setStartState(*move_group.getCurrentState());
	// Pose objetivo
	another_pose.orientation.x = q[0];
	another_pose.orientation.y = q[1];
	another_pose.orientation.z = q[2];
	another_pose.orientation.w = q[3];
	another_pose.position.x =  3.0;
	another_pose.position.y =  0.0;
	another_pose.position.z =  5.0;
	move_group.setPoseTarget(another_pose);
	// Plan
	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("robocol", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");
	// Visualize the plan in RViz
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("next step\n");
	// Move
	move_group.move();
	/* Wait for MoveGroup to recieve and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the robot\n");
	// Now, let's remove the collision object from the world.
	ROS_INFO_NAMED("robocol", "Remove the object from the world");
	std::vector<std::string> object_ids;
	object_ids.push_back(collision_object.id);
	planning_scene_interface.removeCollisionObjects(object_ids);
	// Show text in RViz of status
	visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();
	/* Wait for MoveGroup to recieve and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears\n");
}

void planning_with_constrains(
	moveit::planning_interface::MoveGroupInterface &move_group,
	moveit_visual_tools::MoveItVisualTools &visual_tools,
	const moveit::core::JointModelGroup* joint_model_group,
	moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo\n");
	// Planning with Path Constraints
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Path constraints can easily be specified for a link on the robot.
	// Let's specify a path constraint and a pose goal for our group.
	// First define the path constraint.
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "wrist_link__3";
	ocm.header.frame_id = "base_link";
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	// Now, set it as the path constraint for the group.
	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	move_group.setPathConstraints(test_constraints);

	// We will reuse the old goal that we had and plan to it.
	// Note that this will only work if the current state already
	// satisfies the path constraints. So, we need to set the start
	// state to a new pose.
	robot_state::RobotState start_state(*move_group.getCurrentState());
	geometry_msgs::PoseStamped start_pose = move_group.getCurrentPose();
	geometry_msgs::Pose target_pose;
	target_pose.orientation.w = 0.0;
	target_pose.position.x =  3.0;
	target_pose.position.y =  0.0;
	target_pose.position.z =  5.5;
	start_state.setFromIK(joint_model_group, start_pose.pose);
	move_group.setStartState(start_state);
	printf(" START -> x: %f y: %f z: %f\n", start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
	// Now we will plan to the earlier pose target from the new
	// start state that we have just created.
	move_group.setPoseTarget(target_pose);
	// Planning with constraints can be slow because every sample must call an inverse kinematics solver.
	// Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
	move_group.setPlanningTime(10.0);
	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("robocol", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
	// Visualize the plan in RViz
	visual_tools.deleteAllMarkers();
	visual_tools.publishAxisLabeled(start_pose.pose, "start");
	visual_tools.publishAxisLabeled(target_pose, "goal");
	visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("next step\n");
	// When done with the path constraint be sure to clear it.
	move_group.clearPathConstraints();
}

int main(int argc, char** argv)
{
	// Iniciar nodo de ROS
	ros::init(argc,argv,"inverseKinem");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ROS_INFO("Start.");

	// Interfaz de Movegroup (para interactuar con el brazo)
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	// Interfaz de planeación (Para interactuar con objetos)
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// Estado acutal
	moveit::core::RobotStatePtr current_state;
	// RVIZ VISUAL TOOLS
	// rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	// Planeador para computar y visualizar los planes.
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	// Grupo de modelo de junturas
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	// Publishers
	ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1);
	// Planning Scene Diff Publisher
	ros::WallDuration sleep_t(0.5);
	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
		sleep_t.sleep();
	}
	// Limpiar el mundo de objetos.
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.clear();
	planning_scene_diff_publisher.publish(planning_scene);
	// Configurar Visual Tools
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();
	text_pose.translation().z() = 10.0;
	visual_tools.publishText(text_pose, "ROBOCOL ARM", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();
	// Imprimir el planning frame.
	ROS_INFO_NAMED("robocol", "Planning frame: %s", move_group.getPlanningFrame().c_str());
	// Nombre del end-effector link para este grupo.
	ROS_INFO_NAMED("robocol", "End effector link: %s", move_group.getEndEffectorLink().c_str());
	// Lista de los grupos en el robot:
	ROS_INFO_NAMED("robocol", "Available Planning Groups:");
	std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
	       std::ostream_iterator<std::string>(std::cout, ", "));
	// Posición actual de las junturas.
	current_state = move_group.getCurrentState();
	
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	printf("\n Joints position: ");
	for (int i = 0; i < joint_group_positions.size(); i++)
	{
		std::cout << joint_group_positions.at(i) << ' ';
	}
	printf(" \n");
	// Pose actual.
	geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
	printf("\n Pose -> x: %f, y: %f, z: %f\n", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);	
	// Planear a una pose de objetivo
	planning_to_pose_goal(move_group, visual_tools, joint_model_group, my_plan);
	// Planear a una pose de juntura objetivo
	planning_to_joint_space_goal(move_group, visual_tools, joint_model_group, my_plan, current_state);
	// Planear un camino cartesiano
	planning_cartesian_path(move_group, visual_tools);
	// Parámetros de cinemática inversa
	inverse_kinematics(visual_tools);
	// Manejo de objetos
	object_management(move_group, visual_tools, joint_model_group, my_plan, planning_scene_interface);
	// Borrar marcadores
	visual_tools.deleteAllMarkers();
	// End ROS
	ros::shutdown();
	return(0);
}
