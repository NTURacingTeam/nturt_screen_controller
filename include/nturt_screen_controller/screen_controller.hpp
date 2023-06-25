/**
 * @file nturt_led_controller.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS2 package for controlling dashboard screen.
 */

#ifndef SCREEN_CONTROLLER_HPP
#define SCREEN_CONTROLLER_HPP

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>

// nturt include
#include "nturt_can_config.h"
#include "nturt_can_config_logger-binutil.h"

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for controlling dashboard screen.
 */
class ScreenController : public rclcpp::Node {
 public:
  /// @brief Constructor of screen_controller.
  ScreenController(rclcpp::NodeOptions options);

 private:
  /// @brief Callback function when receiving message from "/from_can_bus".
  void onCan(const std::shared_ptr<can_msgs::msg::Frame> msg);

  /// @brief Timed callback function for periodically checking can receive
  /// timeout.
  void check_can_timer_callback();

  /// @brief ROS2 sbscriber to "/from_can_bus", for receiving can signal.
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  /// @brief Struct for storing can frame data.
  nturt_can_config_logger_rx_t can_rx_;

  /// @brief ROS2 timer for periodically checking can receive timeout.
  rclcpp::TimerBase::SharedPtr check_can_timer_;
};

#endif  // SCREEN_CONTROLLER_HPP
