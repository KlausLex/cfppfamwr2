#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.h>  // Include this for the Pose message

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Get the planner ID from command-line arguments or use a default value
  std::string planner_id = "PRMkConfigDefault";  // Default planner ID
  if (argc > 1) {
    planner_id = argv[1];
  }

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Set the planner ID based on the provided parameter
  move_group_interface.setPlannerId(planner_id);

  // Set a target Pose (cabinet free space top)
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = -0.310;
  target_pose.position.y = -0.925;
  target_pose.position.z = 0.709;
  target_pose.orientation.x = 0;
  target_pose.orientation.y = -0.703;
  target_pose.orientation.z = 0.711;
  target_pose.orientation.w = 0;
  move_group_interface.setPoseTarget(target_pose);

  // Set the planning timeout in seconds
  double planning_timeout = 10.0;  // Change this value as needed

  // Set the planning time for the MoveGroupInterface
  move_group_interface.setPlanningTime(planning_timeout);

  // Set the number of planning attempts
  int num_planning_attempts = 5;  // Change this value as needed
  move_group_interface.setNumPlanningAttempts(num_planning_attempts);


  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
