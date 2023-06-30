#ifndef SCREEN_HPP
#define SCREEN_HPP

// stl include
#include <iostream>
#include <memory>

#ifdef BACKEND_GLFW
// glfw include
#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif
#include <GLFW/glfw3.h>
#elif defined(BACKEND_SDL)
// sdl include
#include <SDL.h>
#include <SDL_opengl.h>
#endif

// imgui include
#include "imgui.h"

// nturt include
#include "nturt_screen_controller/screen_data.hpp"

/**
 * @brief Class for controlling the front of screen.
 *
 * @author
 */
class Screen {
 public:
  /// @brief Constructor.
  Screen(std::shared_ptr<ScreenData> data);

  /**
   * @brief Initialize the screen.
   *
   * @return int 0 if success, 1 if failed.
   */
  int init();

  /// @brief Main loop to update and rander the screen.
  void mainloop();

  /// @brief Cleanup the display.
  void cleanup();

 private:
  /// @brief Display the main driver information.
  void display_driver_information();

  /// @brief Display the sensor data information.
  void display_sensor_data();

#ifdef BACKEND_GLFW
  /// @brief GLFW window.
  GLFWwindow *window_ = nullptr;
#elif defined(BACKEND_SDL)
  /// @brief SDL window.
  SDL_Window *window_ = nullptr;

  /// @brief SDL GL context.
  SDL_GLContext gl_context_;

  /// @brief If main loop is done.
  bool done_ = false;
#endif

  // imgui fonts
  /// @brief ImGui font for speed.
  ImFont *speed_font_;

  /// @brief ImGui font for speed unit.
  ImFont *speed_unit_font_;

  /// @brief ImGui font for battery.
  ImFont *battery_font_;

  /// @brief ImGui font for snesnor data.
  ImFont *sensor_data_font_;

  /// @brief ImGui font for steer angle.
  ImFont *steer_angle_font_;

  /// @brief ImGui font for error/warning message.
  ImFont *error_warning_message_font_;

  /// @brief Shared pointer to screen data.
  std::shared_ptr<ScreenData> data_;
};

#endif  // SCREEN_HPP
