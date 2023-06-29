// stl include
#include <functional>
#include <memory>
#include <thread>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// nturt include
#include "nturt_screen_controller/screen.hpp"
#include "nturt_screen_controller/screen_data.hpp"

using namespace std::chrono_literals;

class ScreenTest : public rclcpp::Node {
 public:
  /// @brief Constructor of ScreenTest.
  ScreenTest(rclcpp::NodeOptions options)
      : Node("screen_test_node", options),
        toggle_screen_timer_(this->create_wall_timer(
            3000ms, std::bind(&ScreenTest::toggle_screen_callback, this))),
        data_(std::make_shared<ScreenData>()),
        screen_(data_),
        screen_thread_(std::bind(&ScreenTest::screen_thread_task, this)) {}

 private:
  void toggle_screen_callback() {
    std::lock_guard<std::mutex> lock(data_->mutex);
    if (data_->show_sensor_data) {
      data_->show_sensor_data = false;
    } else {
      data_->show_sensor_data = true;
    }
  }

  void screen_thread_task() {
    screen_.init();
    screen_.mainloop();
    screen_.cleanup();
  }

  rclcpp::TimerBase::SharedPtr toggle_screen_timer_;

  std::shared_ptr<ScreenData> data_;

  Screen screen_;

  std::thread screen_thread_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto screen_test_node = std::make_shared<ScreenTest>(options);

  executor.add_node(screen_test_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
