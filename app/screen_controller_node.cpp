// std include
#include <memory>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_screen_controller/screen_controller.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto screen_controller_node = std::make_shared<ScreenController>(options);
  screen_controller_node->register_can_callback();

  executor.add_node(screen_controller_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
