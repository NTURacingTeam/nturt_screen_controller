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
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// nturt include
#include "nturt_can_config.h"
#include "nturt_can_config/can_callback_register.hpp"
#include "nturt_can_config_logger-binutil.h"
#include "nturt_ros_interface/msg/system_stats.hpp"
#include "nturt_screen_controller/screen.hpp"
#include "nturt_screen_controller/screen_data.hpp"

using namespace std::chrono_literals;

ScreenController::ScreenController(rclcpp::NodeOptions options)
    : Node("nturt_screen_controller_node", options),
      can_sub_(this->create_subscription<can_msgs::msg::Frame>(
          "/from_can_bus", 50,
          std::bind(&ScreenController::onCan, this, std::placeholders::_1))),
      gps_fix_sub_(this->create_subscription<sensor_msgs::msg::NavSatFix>(
          "/fix", 10,
          std::bind(&ScreenController::onGpsFix, this, std::placeholders::_1))),
      gps_vel_sub_(this->create_subscription<geometry_msgs::msg::TwistStamped>(
          "/vel", 10,
          std::bind(&ScreenController::onGpsVel, this, std::placeholders::_1))),
      system_stats_sub_(
          this->create_subscription<nturt_ros_interface::msg::SystemStats>(
              "system_stats", 10,
              std::bind(&ScreenController::onSystemStats, this,
                        std::placeholders::_1))),
      check_can_timer_(this->create_wall_timer(
          100ms, std::bind(&ScreenController::check_can_timer_callback, this))),
      update_can_data_timer_(this->create_wall_timer(
          100ms,
          std::bind(&ScreenController::update_can_data_timer_callback, this))),
      data_(std::make_shared<ScreenData>()),
      screen_(data_),
      screen_thread_(std::bind(&ScreenController::screen_thread_task, this)) {
  // init can_rx_
  memset(&can_rx_, 0, sizeof(can_rx_));
  nturt_can_config_logger_Check_Receive_Timeout_Init(&can_rx_);
}

void ScreenController::register_can_callback() {
  CanCallbackRegieter::register_callback(
      static_cast<get_tick_t>(std::bind(&ScreenController::get_tick, this)));
  CanCallbackRegieter::register_callback(static_cast<fmon_mono_t>(
      std::bind(&ScreenController::fmon_mono, this, std::placeholders::_1,
                std::placeholders::_2)));
  CanCallbackRegieter::register_callback(static_cast<tout_mono_t>(
      std::bind(&ScreenController::tout_mono, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3)));
}

void ScreenController::onCan(const std::shared_ptr<can_msgs::msg::Frame> msg) {
  nturt_can_config_logger_Receive(&can_rx_, msg->data.data(), msg->id,
                                  msg->dlc);
}

void ScreenController::onGpsFix(
    const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg) {
  data_->gps_position[0] = msg->latitude;
  data_->gps_position[1] = msg->longitude;
  data_->gps_position[2] = msg->altitude;
}

void ScreenController::onGpsVel(
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
  data_->gps_velocity[0] = msg->twist.linear.x;
  data_->gps_velocity[1] = msg->twist.linear.y;
  data_->gps_velocity[2] = msg->twist.linear.z;
}

void ScreenController::onSystemStats(
    const std::shared_ptr<nturt_ros_interface::msg::SystemStats> msg) {
  data_->cpu_temperature = msg->cpu_temperature;
  data_->cpu_usage = msg->cpu_usage;
  data_->memory_usage = msg->memory_usage;
  data_->disk_usage = msg->disk_usage;
  data_->swap_usage = msg->swap_usage;
}

void ScreenController::check_can_timer_callback() {
  nturt_can_config_logger_Check_Receive_Timeout(&can_rx_);
}

void ScreenController::update_can_data_timer_callback() {
  data_->brake = can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Brake_phys;
  data_->accelerator =
      (can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Accelerator_1_phys +
       can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Accelerator_2_phys) /
      2.0;
  data_->speed =
      WHEEL_SPEED_TO_VEHICLE_SPPED_RATIO *
      (can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Left_Wheel_Speed_phys +
       can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Right_Wheel_Speed_phys +
       can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Left_Wheel_Speed_phys +
       can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Right_Wheel_Speed_phys) /
      4.0;
  data_->tire_temperature[0] =
      (can_rx_.FRONT_SENSOR_3.FRONT_SENSOR_Front_Left_Tire_Temperature_2_phys +
       can_rx_.FRONT_SENSOR_3.FRONT_SENSOR_Front_Left_Tire_Temperature_3_phys) /
      2.0;
  data_->tire_temperature[1] =
      (can_rx_.FRONT_SENSOR_3.FRONT_SENSOR_Front_Right_Tire_Temperature_2_phys +
       can_rx_.FRONT_SENSOR_3
           .FRONT_SENSOR_Front_Right_Tire_Temperature_3_phys) /
      2.0;
  data_->tire_temperature[2] =
      (can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Left_Tire_Temperature_2_phys +
       can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Left_Tire_Temperature_3_phys) /
      2.0;
  data_->tire_temperature[3] =
      (can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Right_Tire_Temperature_2_phys +
       can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Right_Tire_Temperature_3_phys) /
      2.0;

  data_->battery_life = 0.5;

  data_->imu_acceleration[0] = can_rx_.IMU_Acceleration.IMU_Acceleration_X_phys;
  data_->imu_acceleration[1] = can_rx_.IMU_Acceleration.IMU_Acceleration_Y_phys;
  data_->imu_acceleration[2] = can_rx_.IMU_Acceleration.IMU_Acceleration_Z_phys;

  data_->error_code[0] = can_rx_.VCU_Status.VCU_Error_Code;
  data_->error_code[1] = can_rx_.REAR_SENSOR_Status.REAR_SENSOR_Error_Code;
  data_->error_code[2] = (can_rx_.INV_Fault_Codes.INV_Post_Fault_Hi >> 16) &
                         can_rx_.INV_Fault_Codes.INV_Post_Fault_Lo;
  data_->error_code[3] = (can_rx_.INV_Fault_Codes.INV_Run_Fault_Hi >> 16) &
                         can_rx_.INV_Fault_Codes.INV_Run_Fault_Lo;

  data_->steer_angle = can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Steer_Angle;
  data_->wheel_speed[0] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Left_Wheel_Speed_phys;
  data_->wheel_speed[1] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Right_Wheel_Speed_phys;
  data_->wheel_speed[2] =
      can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Left_Wheel_Speed_phys;
  data_->wheel_speed[3] =
      can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Right_Wheel_Speed_phys;
  data_->suspension[0] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Left_Suspension_phys;
  data_->suspension[1] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Right_Suspension_phys;
  data_->suspension[2] =
      can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Left_Suspension_phys;
  data_->suspension[3] =
      can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Right_Suspension_phys;
  data_->brake_pressure[0] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Brake_Pressure_phys;
  data_->brake_pressure[1] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Rear_Brake_Pressure_phys;

  data_->imu_angular_velocity[0] =
      can_rx_.IMU_Angular_Velocity.IMU_Angular_Velocity_X_phys;
  data_->imu_angular_velocity[1] =
      can_rx_.IMU_Angular_Velocity.IMU_Angular_Velocity_Y_phys;
  data_->imu_angular_velocity[2] =
      can_rx_.IMU_Angular_Velocity.IMU_Angular_Velocity_Z_phys;
  data_->imu_quaternion[0] = can_rx_.IMU_Quaternion.IMU_Quaternion_W_phys;
  data_->imu_quaternion[1] = can_rx_.IMU_Quaternion.IMU_Quaternion_X_phys;
  data_->imu_quaternion[2] = can_rx_.IMU_Quaternion.IMU_Quaternion_Y_phys;
  data_->imu_quaternion[3] = can_rx_.IMU_Quaternion.IMU_Quaternion_Z_phys;

  data_->motor_temperature = can_rx_.INV_Temperature_Set_3.INV_Motor_Temp_phys;
  data_->inverter_temperature =
      can_rx_.INV_Temperature_Set_2.INV_Control_Board_Temp_phys;
  data_->battery_temperature = 30;
}

void ScreenController::screen_thread_task() {
  screen_.init();
  screen_.mainloop();
  screen_.cleanup();
}

uint32_t ScreenController::get_tick() {
  return static_cast<uint32_t>(now().nanoseconds() / 1000000);
}

void ScreenController::fmon_mono(FrameMonitor_t* mon, uint32_t msgid) {
  (void)msgid;

  if (mon->cycle_error) {
    mon->cycle_error = false;
  }
}

void ScreenController::tout_mono(FrameMonitor_t* mon, uint32_t msgid,
                                 uint32_t lastcyc) {
  if (!mon->cycle_error) {
    mon->cycle_error = true;
    RCLCPP_WARN(get_logger(), "receive timeout: %u, last cycle: %u", msgid,
                lastcyc);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ScreenController)
