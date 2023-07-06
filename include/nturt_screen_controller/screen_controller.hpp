/**
 * @file nturt_led_controller.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS2 package for controlling dashboard screen.
 */

#ifndef SCREEN_CONTROLLER_HPP
#define SCREEN_CONTROLLER_HPP

// glibc include
#include <stdint.h>

// stl include
#include <array>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// nturt include
#include "nturt_can_config.h"
#include "nturt_can_config_logger-binutil.h"
#include "nturt_ros_interface/msg/system_stats.hpp"
#include "nturt_screen_controller/screen.hpp"
#include "nturt_screen_controller/screen_data.hpp"

/* macro ---------------------------------------------------------------------*/
// prameters
#define PAGE_PIN 25

#define WHEEL_SPEED_TO_VEHICLE_SPPED_RATIO 0.0957F
#define MOTOR_SPEED_TO_VEHICLE_SPEED_RATIO 0.0223F

#define NUM_BATTERY_SEGMENT 7
#define NUM_BATTERY_CELL_PER_SEGMENT 12
#define NUM_BATTERY_CELL_PER_FRAME 3

#define FRONT_BOX_ERROR_MASK 0xFFFFFFFBUL
#define REAR_BOX_ERROR_MASK 0xFFFFFFFBUL
#define INVERTER_POST_ERROR_MASK 0xFFFFFFFFUL
#define INVERTER_RUN_ERROR_MASK 0xFFFFFFFFUL

/* typedef -------------------------------------------------------------------*/
// battery data
typedef std::array<std::array<double, NUM_BATTERY_CELL_PER_SEGMENT>,
                   NUM_BATTERY_SEGMENT>
    battery_data_t;

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for controlling dashboard screen.
 */
class ScreenController : public rclcpp::Node {
 public:
  /// @brief Constructor of screen_controller.
  ScreenController(rclcpp::NodeOptions options);

  /// @brief Register codedbc callback function for can signal.
  void register_can_callback();

 private:
  /// @brief Callback function when receiving message from "/from_can_bus".
  void onCan(const std::shared_ptr<can_msgs::msg::Frame> msg);

  /// @brief Callback function when receiving message from "/fix".
  void onGpsFix(const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg);

  /// @brief Callback function when receiving message from "/vel".
  void onGpsVel(const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg);

  /// @brief Callback function when receiving message from "/system_stats".
  void onSystemStats(
      const std::shared_ptr<nturt_ros_interface::msg::SystemStats> msg);

  /// @brief Timed callback function for periodically checking can receive
  /// timeout.
  void check_can_timer_callback();

  /// @brief Timed callback function for periodically copying can_rx_ to data_
  /// for screen to display.
  void update_can_data_timer_callback();

#if !defined(__aarch64__) || defined(__APPLE__)
  /// @brief Timed callback function for periodically copying can_rx_ to data_
  /// for screen to display.
  void toggle_screen_timer_callback();
#endif

  /// @brief Thread task for controlling screen.
  void screen_thread_task();

  /* coder dbc callback function ---------------------------------------------*/
  uint32_t get_tick();

  void fmon_mono(FrameMonitor_t* mon, uint32_t msgid);

  void tout_mono(FrameMonitor_t* mon, uint32_t msgid, uint32_t lastcyc);

  /// @brief ROS2 sbscriber to "/from_can_bus", for receiving can signal.
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  /// @brief ROS2 sbscriber to "/fix", for receiving GPS signal.
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_sub_;

  /// @brief ROS2 sbscriber to "/vel", for receiving GPS signal.
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      gps_vel_sub_;

  /// @brief ROS2 sbscriber to "/system_stats", for receiving system stats
  /// information.
  rclcpp::Subscription<nturt_ros_interface::msg::SystemStats>::SharedPtr
      system_stats_sub_;

  /// @brief ROS2 timer for periodically checking can receive timeout.
  rclcpp::TimerBase::SharedPtr check_can_timer_;

  /// @brief ROS2 timer for periodically copying can_rx_ to data_ for screen to
  /// display.
  rclcpp::TimerBase::SharedPtr update_can_data_timer_;

#if !defined(__aarch64__) || defined(__APPLE__)
  /// @brief ROS2 timer for periodically toggling screen page.
  rclcpp::TimerBase::SharedPtr toggle_screen_timer_;
#endif

  /// @brief Shared pointer to ScreenData for displaying data on screen.
  std::shared_ptr<ScreenData> data_;

  /// @brief Front end of the screen.
  Screen screen_;

  /// @brief Thread for the front end of the screen.
  std::thread screen_thread_;

  /// @brief Struct for storing can frame data.
  nturt_can_config_logger_rx_t can_rx_;

  /// @brief 2D array for storing battery cell voltage.
  battery_data_t battery_cell_voltage_;

  /// @brief 2D array for storing battery cell temperature.
  battery_data_t battery_cell_temperature_;
};

#endif  // SCREEN_CONTROLLER_HPP
