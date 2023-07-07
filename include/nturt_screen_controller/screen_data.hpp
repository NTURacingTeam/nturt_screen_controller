/**
 * @file screen_data.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Data type definition for screen.
 */

#ifndef SCREEN_DATA_HPP
#define SCREEN_DATA_HPP

// glibc include
#include <stdint.h>

// stl include
#include <mutex>
#include <string>
#include <vector>

struct ScreenData {
  /* dirver info -------------------------------------------------------------*/
  double brake;
  double accelerator;

  double speed;
  double tire_temperature[4];

  double battery_life;

  // from 0 ~ 3 are error code and mask for front_box, rear_box, inverter_run,
  // inverter_post
  uint32_t error_code[4];
  /// @brief If error_code & error_mask != 0, it's error, otherwise it's
  /// warning if error_code != 0.
  uint32_t error_mask[4];

  /// @brief Timeout frame name to display on the screen.
  std::vector<std::string> timeout_frame_name;

  /* sensor data -------------------------------------------------------------*/
  double steer_angle;
  double wheel_speed[4];
  double suspension[4];
  double brake_pressure[2];

  double gps_position[3];
  double gps_velocity[3];

  double imu_acceleration[3];
  double imu_angular_velocity[3];
  double imu_quaternion[4];

  double motor_temperature;
  double battery_temperature;

  double inverter_dc_voltage;
  double inverter_control_board_temperature;
  double inverter_hot_spot_temperature;
  uint8_t inverter_state;
  uint8_t inverter_vsm_state;

  uint8_t vcu_state;
  uint16_t rtd_condition;
  uint8_t rear_box_state;

  double cpu_temperature;
  double cpu_usage;
  double memory_usage;
  double disk_usage;
  double swap_usage;

  /* control flag ------------------------------------------------------------*/
  /// @brief Flag for showing sensor data (true) or driver information (false).
  bool show_sensor_data;

  std::mutex mutex;
};

#endif  // SCREEN_DATA_HPP
