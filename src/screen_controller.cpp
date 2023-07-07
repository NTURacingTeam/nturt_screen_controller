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
#include "nturt_can_config/can_callback_register.hpp"
#include "nturt_can_config_logger-binutil.h"
#include "nturt_ros_interface/msg/system_stats.hpp"
#include "nturt_screen_controller/screen.hpp"
#include "nturt_screen_controller/screen_data.hpp"

using namespace std::chrono_literals;

/* Private macro -------------------------------------------------------------*/
/* can frame index for rx receive timeout error ------------------------------*/
#define NUM_FRAME_FRONT 5
#define NUM_FRAME_REAR 3
#define NUM_FRAME_BMS 1
#define NUM_FRAME_INVERTER 5
#define NUM_FRAME \
  (NUM_FRAME_FRONT + NUM_FRAME_REAR + NUM_FRAME_BMS + NUM_FRAME_INVERTER)

// front box frame
#define FRAME_FRONT_BASE 0UL
#define FRAME_FRONT(X) (1UL << (FRAME_FRONT_BASE + X))
#define FRAME_FRONT_MASK ((1UL << (FRAME_FRONT_BASE + NUM_FRAME_FRONT)) - 1UL)

#define VCU_Status_INDEX FRAME_FRONT(0)
#define INV_Command_Message_INDEX FRAME_FRONT(1)
#define FRONT_SENSOR_1_INDEX FRAME_FRONT(2)
#define FRONT_SENSOR_2_INDEX FRAME_FRONT(3)
#define FRONT_SENSOR_3_INDEX FRAME_FRONT(4)

// rear box frame
#define FRAME_REAR_BASE (FRAME_FRONT_BASE + NUM_FRAME_FRONT)
#define FRAME_REAR(X) (1UL << (FRAME_REAR_BASE + X))
#define FRAME_REAR_MASK                                  \
  (((1UL << (FRAME_REAR_BASE + NUM_FRAME_REAR)) - 1UL) - \
   ((1UL << FRAME_REAR_BASE) - 1UL))

#define REAR_SENSOR_Status_INDEX FRAME_REAR(0)
#define REAR_SENSOR_1_INDEX FRAME_REAR(1)
#define REAR_SENSOR_2_INDEX FRAME_REAR(2)

// bms frame
#define FRAME_BMS_BASE (FRAME_REAR_BASE + NUM_FRAME_REAR)
#define FRAME_BMS(X) (1UL << (FRAME_BMS_BASE + X))
#define FRAME_BMS_MASK                                 \
  (((1UL << (FRAME_BMS_BASE + NUM_FRAME_BMS)) - 1UL) - \
   ((1UL << FRAME_BMS_BASE) - 1UL))

#define BMS_Cell_Stats_INDEX FRAME_BMS(0)

// inverter frame
#define FRAME_INVERTER_BASE (FRAME_BMS_BASE + NUM_FRAME_BMS)
#define FRAME_INVERTER(X) (1UL << (FRAME_INVERTER_BASE + X))
#define FRAME_INVERTER_MASK                                      \
  (((1UL << (FRAME_INVERTER_BASE + NUM_FRAME_INVERTER)) - 1UL) - \
   ((1UL << FRAME_INVERTER_BASE) - 1UL))

#define INV_Fast_Info_INDEX FRAME_INVERTER(0)
#define INV_Fault_Codes_INDEX FRAME_INVERTER(1)
#define INV_Internal_States_INDEX FRAME_INVERTER(2)
#define INV_Temperature_Set_2_INDEX FRAME_INVERTER(3)
#define INV_Temperature_Set_3_INDEX FRAME_INVERTER(4)

/// @brief Map to convert can id to frame name.
static const std::map<uint32_t, uint32_t> can_id_to_frame_name = {
    {VCU_Status_CANID, VCU_Status_INDEX},
    {INV_Command_Message_CANID, INV_Command_Message_INDEX},
    {FRONT_SENSOR_1_CANID, FRONT_SENSOR_1_INDEX},
    {FRONT_SENSOR_2_CANID, FRONT_SENSOR_2_INDEX},
    {FRONT_SENSOR_3_CANID, FRONT_SENSOR_3_INDEX},

    {REAR_SENSOR_Status_CANID, REAR_SENSOR_Status_INDEX},
    {REAR_SENSOR_1_CANID, REAR_SENSOR_1_INDEX},
    {REAR_SENSOR_2_CANID, REAR_SENSOR_2_INDEX},

    {BMS_Cell_Stats_CANID, BMS_Cell_Stats_INDEX},

    {INV_Fast_Info_CANID, INV_Fast_Info_INDEX},
    {INV_Fault_Codes_CANID, INV_Fault_Codes_INDEX},
    {INV_Internal_States_CANID, INV_Internal_States_INDEX},
    {INV_Temperature_Set_2_CANID, INV_Temperature_Set_2_INDEX},
    {INV_Temperature_Set_3_CANID, INV_Temperature_Set_3_INDEX},
};

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

  data_->error_mask[0] = FRONT_BOX_ERROR_MASK;
  data_->error_mask[1] = REAR_BOX_ERROR_MASK;
  data_->error_mask[2] = INVERTER_POST_ERROR_MASK;
  data_->error_mask[3] = INVERTER_RUN_ERROR_MASK;
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
  uint32_t id = nturt_can_config_logger_Receive(&can_rx_, msg->data.data(),
                                                msg->id, msg->dlc);

  if (id == BMS_Cell_Stats_CANID) {
    BMS_Cell_Stats_t* bms_cell_stats = &can_rx_.BMS_Cell_Stats;
    int segment_index = bms_cell_stats->BMS_Segment_Index,
        cell_index =
            NUM_BATTERY_CELL_PER_FRAME * bms_cell_stats->BMS_Cell_Index;

    battery_cell_voltage_[segment_index][cell_index] =
        bms_cell_stats->BMS_Cell_Voltage_1_phys;
    battery_cell_voltage_[segment_index][cell_index + 1] =
        bms_cell_stats->BMS_Cell_Voltage_2_phys;
    battery_cell_voltage_[segment_index][cell_index + 2] =
        bms_cell_stats->BMS_Cell_Voltage_3_phys;

    battery_cell_temperature_[segment_index][cell_index] =
        bms_cell_stats->BMS_Cell_Temperature_1;
    battery_cell_temperature_[segment_index][cell_index + 1] =
        bms_cell_stats->BMS_Cell_Temperature_2;
    battery_cell_temperature_[segment_index][cell_index + 2] =
        bms_cell_stats->BMS_Cell_Temperature_3;
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
  data_->error_code[2] = (can_rx_.INV_Fault_Codes.INV_Post_Fault_Hi >> 16) |
                         can_rx_.INV_Fault_Codes.INV_Post_Fault_Lo;
  data_->error_code[3] = (can_rx_.INV_Fault_Codes.INV_Run_Fault_Hi >> 16) |
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
  data_->battery_life = 0.5;
  data_->battery_temperature = *std ::max_element(
      &battery_cell_temperature_[0][0],
      &battery_cell_temperature_[NUM_BATTERY_SEGMENT - 1]
                                [NUM_BATTERY_CELL_PER_SEGMENT - 1] +
          1);

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
  if (can_rx_error_ & FRAME_FRONT_MASK) {
    data_->timeout_frame_name.push_back("Front Box");
  }
  if (can_rx_error_ & FRAME_REAR_MASK) {
    data_->timeout_frame_name.push_back("Rear Box");
  }
  if (can_rx_error_ & FRAME_BMS_MASK) {
    data_->timeout_frame_name.push_back("BMS");
  }
  if (can_rx_error_ & FRAME_INVERTER_MASK) {
    data_->timeout_frame_name.push_back("Inverter");
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

void ScreenController::fmon_mono(FrameMonitor_t* mon, uint32_t msgid) {
  if (mon->cycle_error) {
    mon->cycle_error = false;

    if (can_id_to_frame_name.find(msgid) != can_id_to_frame_name.end()) {
      can_rx_error_ &= ~can_id_to_frame_name.at(msgid);
    }
  }
}

void ScreenController::tout_mono(FrameMonitor_t* mon, uint32_t msgid,
                                 uint32_t lastcyc) {
  if (!mon->cycle_error) {
    mon->cycle_error = true;

    if (can_id_to_frame_name.find(msgid) != can_id_to_frame_name.end()) {
      can_rx_error_ |= can_id_to_frame_name.at(msgid);
      RCLCPP_WARN(get_logger(), "receive timeout: %u, last cycle: %u", msgid,
                  lastcyc);
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ScreenController)
