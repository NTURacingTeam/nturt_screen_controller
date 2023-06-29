// glibc include
#include <stdint.h>

// stl include
#include <mutex>

#ifndef SCREEN_DATA_HPP
#define SCREEN_DATA_HPP

struct ScreenData {
  /* dirver info -------------------------------------------------------------*/
  double brake;
  double accelerator;

  double speed;
  double tire_temperature[4];

  double battery_life;

  double imu_acceleration[3];

  // from 0 ~ 3 are error code and mask for front_box, rear_box, inverter_run,
  // inverter_post
  uint32_t error_code[4];
  /// @brief If error_code & error_mask != 0, it's error, otherwise it's
  /// warning if error_code != 0.
  uint32_t error_mask[4];

  /* sensor data -------------------------------------------------------------*/
  double steer_angle;
  double wheel_speed[4];
  double suspension[4];
  double brake_pressure[2];

  double gps_position[3];
  double gps_velocity[3];

  double imu_angular_velocity[3];
  double imu_quaternion[4];

  double motor_temperature;
  double inverter_temperature;
  double battery_temperature;

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
