#include "waypoint_navigator/WaypointNavigator.hpp"

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaypointNavigator>("wp_navigator");
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->doWork(node);
  }

  rclcpp::shutdown();
  return 0;
}