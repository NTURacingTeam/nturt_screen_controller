#include "nturt_screen_controller/screen_controller.hpp"

// glibc include
#include <stdint.h>
#include <string.h>

// std include
#include <chrono>
#include <functional>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>

// nturt include
#include "nturt_can_config.h"
#include "nturt_can_config_logger-binutil.h"

/* callback function prototype -----------------------------------------------*/
extern "C" {
uint32_t __get__tick__();
void _FMon_MONO_nturt_can_config(FrameMonitor_t* mon, uint32_t msgid);
void _TOut_MONO_nturt_can_config(FrameMonitor_t* mon, uint32_t msgid,
                                 uint32_t lastcyc);
}

using namespace std::chrono_literals;

ScreenController::ScreenController(rclcpp::NodeOptions options)
    : Node("nturt_screen_controller_node", options),
      can_sub_(this->create_subscription<can_msgs::msg::Frame>(
          "/from_can_bus", 10,
          std::bind(&ScreenController::onCan, this, std::placeholders::_1))),
      check_can_timer_(this->create_wall_timer(
          100ms,
          std::bind(&ScreenController::check_can_timer_callback, this))) {
  // init can_rx_
  memset(&can_rx_, 0, sizeof(can_rx_));
  nturt_can_config_logger_Check_Receive_Timeout_Init(&can_rx_);
}

void ScreenController::onCan(const std::shared_ptr<can_msgs::msg::Frame> msg) {
  nturt_can_config_logger_Receive(&can_rx_, msg->data.data(), msg->id,
                                  msg->dlc);
}

void ScreenController::check_can_timer_callback() {
  nturt_can_config_logger_Check_Receive_Timeout(&can_rx_);
}

/* callback function ---------------------------------------------------------*/
// coderdbc callback function for getting current time in ms
uint32_t __get__tick__() {
  return static_cast<uint32_t>(rclcpp::Clock().now().nanoseconds() / 1000000);
}

// coderdbc callback function called when receiving a new frame
void _FMon_MONO_nturt_can_config(FrameMonitor_t* mon, uint32_t msgid) {
  (void)msgid;

  if (mon->cycle_error) {
    mon->cycle_error = false;
  }
}

// coderdbc callback function called when reception timeout
void _TOut_MONO_nturt_can_config(FrameMonitor_t* mon, uint32_t msgid,
                                 uint32_t lastcyc) {
  if (!mon->cycle_error) {
    mon->cycle_error = true;
    RCLCPP_WARN(rclcpp::get_logger("nturt_screen_controller_node"),
                "receive timeout: %u, last cycle: %u", msgid, lastcyc);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ScreenController)
