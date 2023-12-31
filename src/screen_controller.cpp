#include "nturt_screen_controller/screen_controller.hpp"

// glibc include
#include <stdint.h>
#include <string.h>

// std include
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>

#if defined(__aarch64__) && !defined(__APPLE__)
// gpio include
#include <wiringPi.h>
#endif

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// nturt include
#include "nturt_can_config.h"
#include "nturt_can_config/battery_utils.hpp"
#include "nturt_can_config/can_callback_register.hpp"
#include "nturt_can_config/can_timeout_monitor.hpp"
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
              "/system_stats", 10,
              std::bind(&ScreenController::onSystemStats, this,
                        std::placeholders::_1))),
      check_can_timer_(this->create_wall_timer(
          100ms, std::bind(&ScreenController::check_can_timer_callback, this))),
      update_can_data_timer_(this->create_wall_timer(
          100ms,
          std::bind(&ScreenController::update_can_data_timer_callback, this))),
#if !defined(__aarch64__) || defined(__APPLE__)
      toggle_screen_timer_(this->create_wall_timer(
          5s,
          std::bind(&ScreenController::toggle_screen_timer_callback, this))),
#endif
      data_(std::make_shared<ScreenData>()),
      screen_(data_),
      screen_thread_(std::bind(&ScreenController::screen_thread_task, this)) {
#if defined(__aarch64__) && !defined(__APPLE__)
  // initiate wiringpi gpio
  wiringPiSetup();
  pinMode(PAGE_PIN, OUTPUT);
#endif

  // init can_rx_
  memset(&can_rx_, 0, sizeof(can_rx_));
  nturt_can_config_logger_Check_Receive_Timeout_Init(&can_rx_);

  // set error mask
  data_->error_mask[0] = FRONT_BOX_ERROR_MASK;
  data_->error_mask[1] = REAR_BOX_ERROR_MASK;
  data_->error_mask[2] = INVERTER_POST_ERROR_MASK;
  data_->error_mask[3] = INVERTER_RUN_ERROR_MASK;

  // reserve ping string
  data_->ping.reserve(20);
}

void ScreenController::register_can_callback() {
  CanCallbackRegieter::register_callback(
      static_cast<get_tick_t>(std::bind(&ScreenController::get_tick, this)));
}

void ScreenController::onCan(const std::shared_ptr<can_msgs::msg::Frame> msg) {
  uint32_t id = nturt_can_config_logger_Receive(&can_rx_, msg->data.data(),
                                                msg->id, msg->dlc);

  if (id == BMS_Cell_Stats_CANID) {
    battery_data_.update(&can_rx_.BMS_Cell_Stats);
  }
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

  // if network not connected
  if (msg->ping == 0) {
    data_->ping = "Not Connected!";
  } else {
    sprintf(data_->ping.data(), "%g ms", msg->ping);
  }

  // if wifi not connected
  if (msg->wifi_strength == 0) {
    data_->wifi_ssid = "Not Connected!";
    data_->wifi_strength = "N/A";
  } else {
    data_->wifi_ssid = msg->wifi_ssid;
    if (msg->wifi_strength > -50) {
      data_->wifi_strength = "Excellent";
    } else if (msg->wifi_strength > -65) {
      data_->wifi_strength = "Good";
    } else if (msg->wifi_strength > -80) {
      data_->wifi_strength = "Fair";
    } else {
      data_->wifi_strength = "Weak";
    }
  }
}

void ScreenController::check_can_timer_callback() {
  nturt_can_config_logger_Check_Receive_Timeout(&can_rx_);
}

void ScreenController::update_can_data_timer_callback() {
#if defined(__aarch64__) && !defined(__APPLE__)
  data_->show_sensor_data = digitalRead(PAGE_PIN);
#endif

  // pedals
  //   data_->brake = can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Brake_phys;
  data_->brake =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Brake_Pressure_phys / 20;
  data_->accelerator =
      (can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Accelerator_1_phys +
       can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Accelerator_2_phys) /
      2.0;

  // speed
  //   data_->speed =
  //       WHEEL_SPEED_TO_VEHICLE_SPPED_RATIO *
  //       (can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Left_Wheel_Speed_phys +
  //        can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Right_Wheel_Speed_phys +
  //        can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Left_Wheel_Speed_phys +
  //        can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Right_Wheel_Speed_phys) /
  //       4.0;
  data_->speed = std::abs(can_rx_.INV_Fast_Info.INV_Fast_Motor_Speed) *
                 MOTOR_SPEED_TO_VEHICLE_SPEED_RATIO;

  // tire temperature
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

  // error code
  data_->error_code[0] = can_rx_.VCU_Status.VCU_Error_Code;
  data_->error_code[1] = can_rx_.REAR_SENSOR_Status.REAR_SENSOR_Error_Code;
  data_->error_code[2] = (can_rx_.INV_Fault_Codes.INV_Post_Fault_Hi << 16) |
                         can_rx_.INV_Fault_Codes.INV_Post_Fault_Lo;
  data_->error_code[3] = (can_rx_.INV_Fault_Codes.INV_Run_Fault_Hi << 16) |
                         can_rx_.INV_Fault_Codes.INV_Run_Fault_Lo;

  // steer andle
  data_->steer_angle = can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Steer_Angle;
  data_->wheel_speed[0] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Left_Wheel_Speed_phys;
  data_->wheel_speed[1] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Right_Wheel_Speed_phys;
  data_->wheel_speed[2] =
      can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Left_Wheel_Speed_phys;
  data_->wheel_speed[3] =
      can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Right_Wheel_Speed_phys;

  // suspension travel
  data_->suspension[0] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Left_Suspension_phys;
  data_->suspension[1] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Right_Suspension_phys;
  data_->suspension[2] =
      can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Left_Suspension_phys;
  data_->suspension[3] =
      can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Right_Suspension_phys;

  // brake pressure
  data_->brake_pressure[0] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Brake_Pressure_phys;
  data_->brake_pressure[1] =
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Rear_Brake_Pressure_phys;

  // imu
  data_->imu_acceleration[0] = can_rx_.IMU_Acceleration.IMU_Acceleration_X_phys;
  data_->imu_acceleration[1] = can_rx_.IMU_Acceleration.IMU_Acceleration_Y_phys;
  data_->imu_acceleration[2] = can_rx_.IMU_Acceleration.IMU_Acceleration_Z_phys;
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

  // battery
  double voltage = battery_data_.average_voltage();
  double current = can_rx_.INV_Current_Info.INV_DC_Bus_Current_phys;
  data_->battery_life = state_of_charge(voltage, current);

  data_->battery_temperature = battery_data_.maximum_temperature();

  // motor
  data_->motor_temperature = can_rx_.INV_Temperature_Set_3.INV_Motor_Temp_phys;

  // inverter
  data_->inverter_dc_voltage =
      can_rx_.INV_Fast_Info.INV_Fast_DC_Bus_Voltage_phys;
  data_->inverter_control_board_temperature =
      can_rx_.INV_Temperature_Set_2.INV_Control_Board_Temp_phys;
  data_->inverter_hot_spot_temperature =
      can_rx_.INV_Temperature_Set_3.INV_Hot_Spot_Temp_phys;
  data_->inverter_state = can_rx_.INV_Internal_States.INV_Inverter_State;
  data_->inverter_vsm_state = can_rx_.INV_Internal_States.INV_VSM_State;

  // status
  data_->vcu_state = can_rx_.VCU_Status.VCU_Status;
  data_->rtd_condition = can_rx_.VCU_Status.VCU_RTD_Condition;
  data_->rear_box_state = can_rx_.REAR_SENSOR_Status.REAR_SENSOR_Status;

  // can rx timeout
  data_->timeout_frame_name.clear();
  if (can_timeout_monior::can_rx_error & FRAME_FRONT_MASK) {
    data_->timeout_frame_name.push_back("Front Box");
  }
  if (can_timeout_monior::can_rx_error & FRAME_REAR_MASK) {
    data_->timeout_frame_name.push_back("Rear Box");
  }
  if (can_timeout_monior::can_rx_error & FRAME_BMS_MASK) {
    data_->timeout_frame_name.push_back("BMS");
  }
  if (can_timeout_monior::can_rx_error & FRAME_INVERTER_MASK) {
    data_->timeout_frame_name.push_back("Inverter");
  }
  if (can_timeout_monior::can_rx_error & FRAME_IMU_MASK) {
    data_->timeout_frame_name.push_back("IMU");
  }
}

#if !defined(__aarch64__) || defined(__APPLE__)
void ScreenController::toggle_screen_timer_callback() {
  if (data_->show_sensor_data) {
    data_->show_sensor_data = false;
  } else {
    data_->show_sensor_data = true;
  }
}
#endif

void ScreenController::screen_thread_task() {
  screen_.init();
  screen_.mainloop();
  screen_.cleanup();
}

uint32_t ScreenController::get_tick() {
  return static_cast<uint32_t>(now().nanoseconds() / 1000000);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ScreenController)
