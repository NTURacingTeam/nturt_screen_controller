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
  double battery_life;

  double imu_acceleration[3];

  double motor_temperature;
  double inverter_temperature;
  double battery_temperature;
  double cpu_temperature;

  double cpu_usage;
  double memory_usage;

  uint32_t error_code[4];

  /* sensor data -------------------------------------------------------------*/
  double steer_angle;
  double wheel_speed[4];
  double tire_temperature[4];
  double suspension_travel[4];
  double brake_oil_pressure[2];

  double gps_position[3];
  double gps_velocity[3];

  double imu_angular_velocity[3];
  double imu_quaternion[4];

  /* control flag ------------------------------------------------------------*/
  bool checkmode;

  std::mutex data_mutex;
};

#endif  // SCREEN_DATA_HPP
