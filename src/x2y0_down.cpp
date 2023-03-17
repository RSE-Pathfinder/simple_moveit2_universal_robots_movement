#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <math.h>


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

tf2::Quaternion myQuaternion;

double roll   = M_PI * 0.0;
double pitch  = M_PI * -3/5;
double yaw    = M_PI * 1.0;

myQuaternion.setRPY(roll, pitch, yaw);
myQuaternion = myQuaternion.normalize();

// Set a target Pose
geometry_msgs::msg::Pose msg;
msg.position.x    = 0.65;
msg.position.y    = 0.0;
msg.position.z    = 0.12;
msg.orientation.w = myQuaternion.getW();
msg.orientation.x = myQuaternion.getX();
msg.orientation.y = myQuaternion.getY();
msg.orientation.z = myQuaternion.getZ();
move_group_interface.setPoseTarget(msg);

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
  RCLCPP_ERROR(logger, "Planing failed!");
}

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
