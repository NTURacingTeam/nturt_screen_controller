#include "nturt_screen_controller/screen.hpp"

// stl include
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>

// ros2 include
#include <ament_index_cpp/get_package_share_directory.hpp>

#ifdef BACKEND_GLFW
// glfw include
#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif
#include <GLFW/glfw3.h>

#include "imgui_impl_glfw.h"
#elif defined(BACKEND_SDL)
// sdl include
#include <SDL.h>
#include <SDL_opengl.h>

#include "imgui_impl_sdl2.h"
#endif

// imgui include
#include "imgui.h"
#include "imgui_impl_opengl2.h"

// nturt include
#include "nturt_screen_controller/screen_data.hpp"

/* static function declarations ----------------------------------------------*/
#ifdef BACKEND_GLFW
static void glfw_error_callback(int error, const char *description);
#endif

/* static variable -----------------------------------------------------------*/
static const std::map<uint8_t, std::string> status_map = {
    {0, "Init"}, {1, "Ready"}, {2, "RTD"}, {3, "Running"}, {4, "Error"}};

Screen::Screen(std::shared_ptr<ScreenData> data) : data_(data) {}

int Screen::init() {
#ifdef BACKEND_GLFW
  /* glfw initialization -----------------------------------------------------*/
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) {
    return 1;
  }

  // create full screen window with graphics context
  window_ = glfwCreateWindow(800, 480, "NTURT Screen", nullptr, nullptr);
  if (window_ == nullptr) {
    return 1;
  }

  glfwMakeContextCurrent(window_);
  // enable vsync
  glfwSwapInterval(1);
#elif defined(BACKEND_SDL)
  /* sdl initialization ------------------------------------------------------*/
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) !=
      0) {
    printf("Error: %s\n", SDL_GetError());
    return 1;
  }

  // From 2.0.18: Enable native IME.
#ifdef SDL_HINT_IME_SHOW_UI
  SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
#endif

  // setup window
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_WindowFlags window_flags =
      (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE |
                        SDL_WINDOW_ALLOW_HIGHDPI);
  window_ = SDL_CreateWindow("NTURT Screen", SDL_WINDOWPOS_CENTERED,
                             SDL_WINDOWPOS_CENTERED, 800, 480, window_flags);
  gl_context_ = SDL_GL_CreateContext(window_);
  SDL_GL_MakeCurrent(window_, gl_context_);
  // full screen
  SDL_SetWindowFullscreen(window_, SDL_WINDOW_FULLSCREEN);
  // hide cursor
  SDL_ShowCursor(SDL_DISABLE);
  // enable vsync
  SDL_GL_SetSwapInterval(1);
#endif

  /* imgui initialization ----------------------------------------------------*/
  // setup dear imgui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard |
                    ImGuiConfigFlags_DockingEnable |
                    ImGuiConfigFlags_ViewportsEnable;

  // when viewports are enabled, tweak WindowRounding/WindowBg so platform
  // windows can look identical to regular ones.
  ImGuiStyle &style = ImGui::GetStyle();
  style.WindowRounding = 0.0f;
  style.Colors[ImGuiCol_WindowBg].w = 1.0f;

  // setup dear imgui style
  ImGui::StyleColorsDark();

  // setup platform/renderer backends
#ifdef BACKEND_GLFW
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
#elif defined(BACKEND_SDL)
  ImGui_ImplSDL2_InitForOpenGL(window_, gl_context_);
#endif
  ImGui_ImplOpenGL2_Init();

  // load fonts
  std::string font_path =
      ament_index_cpp::get_package_share_directory("nturt_screen_controller") +
      "/fonts/";
  speed_font_ = io.Fonts->AddFontFromFileTTF(
      (font_path + "HelveticaNeue.ttc").c_str(), 160.0f);
  speed_unit_font_ = io.Fonts->AddFontFromFileTTF(
      (font_path + "Cousine-Regular.ttf").c_str(), 25.0f);
  battery_font_ = io.Fonts->AddFontFromFileTTF(
      (font_path + "Cousine-Regular.ttf").c_str(), 50.0f);
  sensor_data_font_ = io.Fonts->AddFontFromFileTTF(
      (font_path + "Cousine-Regular.ttf").c_str(), 15.0f);
  steer_angle_font_ = io.Fonts->AddFontFromFileTTF(
      (font_path + "Karla-Regular.ttf").c_str(), 30.0f);
  error_warning_message_font_ = io.Fonts->AddFontFromFileTTF(
      (font_path + "DroidSans.ttf").c_str(), 30.0f);

  return 0;
}

// tire temperature color function
void get_tire_temperature_color(float temperature, ImVec2 pos1, ImVec2 pos2) {
  if (temperature >= 30 && temperature < 40) {
    // grey
    ImGui::GetWindowDrawList()->AddRectFilled(
        pos1, pos2, IM_COL32(128, 128, 128, 200), 5.0f);
  } else if (temperature >= 40 && temperature < 50) {
    // blue
    ImGui::GetWindowDrawList()->AddRectFilled(
        pos1, pos2, IM_COL32(122, 184, 204, 200), 5.0f);
  } else if (temperature >= 50 && temperature < 60) {
    // green
    ImGui::GetWindowDrawList()->AddRectFilled(
        pos1, pos2, IM_COL32(103, 207, 83, 200), 5.0f);
  } else if (temperature >= 60 && temperature < 70) {
    // yellow
    ImGui::GetWindowDrawList()->AddRectFilled(
        pos1, pos2, IM_COL32(242, 209, 22, 200), 5.0f);
  } else if (temperature < 30) {
    ImGui::SetCursorPosX(pos1.x + 7);
    ImGui::SetCursorPosY(pos1.y + 7);
    ImGui::Text("L");
  } else {
    // red
    ImGui::GetWindowDrawList()->AddRectFilled(pos1, pos2,
                                              IM_COL32(235, 80, 33, 200), 5.0f);
  }
}

void Screen::mainloop() {
#ifdef BACKEND_GLFW
  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();
#elif defined(BACKEND_SDL)
  while (!done_) {
    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to
    // tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to
    // your main application, or clear/overwrite your copy of the mouse data.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input
    // data to your main application, or clear/overwrite your copy of the
    // keyboard data. Generally you may always pass all inputs to dear imgui,
    // and hide them from your application based on those two flags.
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT ||
          (event.type == SDL_WINDOWEVENT &&
           event.window.event == SDL_WINDOWEVENT_CLOSE &&
           event.window.windowID == SDL_GetWindowID(window_))) {
        done_ = true;
      }
    }
#endif

    // start dear imgui frame
    ImGui_ImplOpenGL2_NewFrame();
#ifdef BACKEND_GLFW
    ImGui_ImplGlfw_NewFrame();
#elif defined(BACKEND_SDL)
    ImGui_ImplSDL2_NewFrame();
#endif
    ImGui::NewFrame();

    // display
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(800, 480));
    ImGui::Begin("NTURT Screen", NULL,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoNav |
                     ImGuiWindowFlags_NoBackground);

    // background
    ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(0, 0), ImVec2(800, 480),
                                              IM_COL32(107, 113, 115, 150));

    // draw screen
    if (!data_->show_sensor_data) {
      display_driver_information();
    } else {
      display_sensor_data();
    }

    /* render ----------------------------------------------------------------*/
    ImGui::Render();

    ImVec4 clear_color = ImVec4(166.0f / 255.0f, 166.0f / 255.0f,
                                166.0f / 255.0f, 250.0f / 255.0f);
#ifdef BACKEND_GLFW
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
                 clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    // update and render additional platform windows
    // platform functions may change the current OpenGL context, so we
    // save/restore it to make it easier to paste this code elsewhere.
    GLFWwindow *backup_current_context = glfwGetCurrentContext();
    ImGui::UpdatePlatformWindows();
    ImGui::RenderPlatformWindowsDefault();
    glfwMakeContextCurrent(backup_current_context);

    glfwSwapBuffers(window_);
#elif defined(BACKEND_SDL)
    ImGuiIO &io = ImGui::GetIO();
    glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
                 clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    // update and render additional platform windows
    // platform functions may change the current OpenGL context, so we
    // save/restore it to make it easier to paste this code elsewhere.
    SDL_Window *backup_current_window = SDL_GL_GetCurrentWindow();
    SDL_GLContext backup_current_context = SDL_GL_GetCurrentContext();
    ImGui::UpdatePlatformWindows();
    ImGui::RenderPlatformWindowsDefault();
    SDL_GL_MakeCurrent(backup_current_window, backup_current_context);

    SDL_GL_SwapWindow(window_);
#endif
  }
}

void Screen::cleanup() {
  // cleanup
  ImGui_ImplOpenGL2_Shutdown();
#ifdef BACKEND_GLFW
  ImGui_ImplGlfw_Shutdown();
#elif defined(BACKEND_SDL)
  ImGui_ImplSDL2_Shutdown();
#endif
  ImGui::DestroyContext();

#ifdef BACKEND_GLFW
  glfwDestroyWindow(window_);
  glfwTerminate();
#elif defined(BACKEND_SDL)
  SDL_GL_DeleteContext(gl_context_);
  SDL_DestroyWindow(window_);
  SDL_Quit();
#endif
}

void Screen::display_driver_information() {
  std::lock_guard<std::mutex> lock(data_->mutex);

  // variables for error/warning massage
  bool error_flag = false;
  bool warning_flag = false;

  ImVec4 error_color = {1.0f, 0.0f, 0.0f, 1.0f},
         warn_color = {1.0f, 1.0f, 0.0f, 1.0f}, color;

  // error/warning message
  ImGui::PushFont(error_warning_message_font_);

  if (data_->error_code[0]) {
    color = warn_color;
    warning_flag = true;
    if (data_->error_code[0] & data_->error_mask[0]) {
      color = error_color;
      error_flag = true;
    }

    ImGui::SetCursorPos(ImVec2(100, 20));
    ImGui::TextColored(color, "Front Box: 0x%X", data_->error_code[0]);
  }
  if (data_->error_code[1]) {
    color = warn_color;
    warning_flag = true;
    if (data_->error_code[1] & data_->error_mask[1]) {
      color = error_color;
      error_flag = true;
    }

    ImGui::SetCursorPos(ImVec2(100, 60));
    ImGui::TextColored(color, "Rear Box:  0x%X", data_->error_code[1]);
  }
  if (data_->error_code[2]) {
    color = warn_color;
    warning_flag = true;
    if (data_->error_code[2] & data_->error_mask[2]) {
      color = error_color;
      error_flag = true;
    }

    ImGui::SetCursorPos(ImVec2(350, 20));
    ImGui::TextColored(color, "Inv Post: 0x%X", data_->error_code[2]);
  }
  if (data_->error_code[3]) {
    color = warn_color;
    warning_flag = true;
    if (data_->error_code[3] & data_->error_mask[3]) {
      color = error_color;
      error_flag = true;
    }

    ImGui::SetCursorPos(ImVec2(350, 60));
    ImGui::TextColored(color, "Inv Run: 0x%X", data_->error_code[3]);
  }

  ImGui::PopFont();

  // error/warning simbol
  if (error_flag) {
    ImVec2 p = ImGui::GetWindowPos();
    ImVec2 a = ImVec2(p.x + 25, p.y + 25);
    ImVec2 b = ImVec2(p.x + 85, p.y + 85);
    ImVec2 c = ImVec2(p.x + 85, p.y + 25);
    ImVec2 d = ImVec2(p.x + 25, p.y + 85);
    ImGui::GetWindowDrawList()->AddLine(a, b, IM_COL32(255, 0, 0, 255), 15.0f);
    ImGui::GetWindowDrawList()->AddLine(c, d, IM_COL32(255, 0, 0, 255), 15.0f);
  } else if (warning_flag) {
    ImVec2 a = ImVec2(15, 91);
    ImVec2 b = ImVec2(85, 91);
    ImVec2 c = ImVec2(50, 25);
    ImGui::GetWindowDrawList()->AddTriangleFilled(a, b, c,
                                                  IM_COL32(255, 255, 0, 255));
  }

  // battery life
  double battery_life_percentage = data_->battery_life * 100.0;
  if (battery_life_percentage > 75) {
    // green battery
    ImGui::GetWindowDrawList()->AddRectFilled(
        ImVec2(762, 86), ImVec2(762 - battery_life_percentage * 1.52, 28),
        IM_COL32(0, 255, 0, 255));
  } else if (battery_life_percentage > 50) {
    // yellow battery
    ImGui::GetWindowDrawList()->AddRectFilled(
        ImVec2(762, 28), ImVec2(762 - battery_life_percentage * 1.52, 86),
        IM_COL32(255, 255, 0, 255));
  } else if (battery_life_percentage > 25) {
    // orange battery
    ImGui::GetWindowDrawList()->AddRectFilled(
        ImVec2(762, 28), ImVec2(762 - battery_life_percentage * 1.52, 86),
        IM_COL32(255, 165, 0, 255));
  } else {
    // red battery
    ImGui::GetWindowDrawList()->AddRectFilled(
        ImVec2(762, 28), ImVec2(762 - battery_life_percentage * 1.52, 86),
        IM_COL32(255, 0, 0, 255));
  }

  // battery shape
  ImGui::GetWindowDrawList()->AddRect(ImVec2(610, 28), ImVec2(762, 86),
                                      IM_COL32(0, 0, 0, 255), 5.0f, 0, 3.0f);
  ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(600, 45), ImVec2(610, 69),
                                            IM_COL32(0, 0, 0, 255));

  // battery text
  ImGui::SetCursorPos(ImVec2(650, 35));
  ImGui::PushFont(battery_font_);
  ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
  ImGui::Text("%.0f%%", battery_life_percentage);
  ImGui::PopStyleColor();
  ImGui::PopFont();

  // brake
  ImGui::GetWindowDrawList()->AddRectFilled(
      ImVec2(ImGui::GetWindowPos().x + 5 + 30,
             ImGui::GetWindowPos().y + 5 + 240),
      ImVec2(ImGui::GetWindowPos().x + 35 + 30,
             ImGui::GetWindowPos().y + 217 + 240),
      IM_COL32(0, 0, 0, 255));
  ImGui::GetWindowDrawList()->AddRectFilled(
      ImVec2(ImGui::GetWindowPos().x + 5 + 30,
             ImGui::GetWindowPos().y + 217 - 200 * data_->brake + 240),
      ImVec2(ImGui::GetWindowPos().x + 35 + 30,
             ImGui::GetWindowPos().y + 217 + 240),
      IM_COL32(192, 0, 0, 255));

  // accelerator
  ImGui::GetWindowDrawList()->AddRectFilled(
      ImVec2(ImGui::GetWindowPos().x + 5 + 30 + 50,
             ImGui::GetWindowPos().y + 5 + 240),
      ImVec2(ImGui::GetWindowPos().x + 35 + 30 + 50,
             ImGui::GetWindowPos().y + 217 + 240),
      IM_COL32(0, 0, 0, 255));
  ImGui::GetWindowDrawList()->AddRectFilled(
      ImVec2(ImGui::GetWindowPos().x + 5 + 30 + 50,
             ImGui::GetWindowPos().y + 217 - 200 * data_->accelerator + 240),
      ImVec2(ImGui::GetWindowPos().x + 35 + 30 + 50,
             ImGui::GetWindowPos().y + 217 + 240),
      IM_COL32(0, 192, 0, 255));

  // speed background
  ImGui::GetWindowDrawList()->AddCircleFilled(
      ImVec2(ImGui::GetWindowPos().x + 240 + 158,
             ImGui::GetWindowPos().y + 240 + 130),
      235, IM_COL32(68, 144, 196, 255));

  // speed background line
  for (int i = 0; i < 13; i++) {
    // calculate the angle of the line
    double angle_background =
        (i * 10.0f / 120.0f) * 225.0f * 3.1415926f / 180.0f;
    // set the start of the line
    ImVec2 start_background =
        ImVec2(398 + 205 * cos(158.5 * 3.1415926f / 180.0f + angle_background),
               370 + 205 * sin(158.5 * 3.1415926f / 180.0f + angle_background));
    // set the end of the line
    ImVec2 end_background =
        ImVec2(398 + 235 * cos(158.5 * 3.1415926f / 180.0f + angle_background),
               370 + 235 * sin(158.5 * 3.1415926f / 180.0f + angle_background));
    // draw the line
    ImGui::GetWindowDrawList()->AddLine(start_background, end_background,
                                        ImColor(255, 255, 255), 3.0f);
  }

  // speed pointer
  // calculate the angle of the line
  double angle = (data_->speed / 120.0f) * 225.0f * 3.1415926f / 180.0f;
  double temp = 0.0f;
  while (temp < angle) {
    // set the start of the line
    ImVec2 start = ImVec2(398 + 200 * cos(158.5 * 3.1415926f / 180.0f + temp),
                          370 + 200 * sin(158.5 * 3.1415926f / 180.0f + temp));
    // set the end of the line
    ImVec2 end = ImVec2(398 + 238 * cos(158.5 * 3.1415926f / 180.0f + temp),
                        370 + 238 * sin(158.5 * 3.1415926f / 180.0f + temp));
    // draw the line
    ImGui::GetWindowDrawList()->AddLine(start, end, ImColor(255, 0, 0), 1.0f);
    temp += 0.001f;
  }

  // speed text
  ImGui::PushFont(speed_font_);
  char buffer[4];
  snprintf(buffer, sizeof(buffer), "%.0f", data_->speed);

  // calculate the position of the text
  ImVec2 speed_text_size = ImGui::CalcTextSize(buffer);
  double speed_text_x = (308 - speed_text_size.x) / 2.0f;
  ImGui::SetCursorPos(ImVec2(245 + speed_text_x, 240));
  ImGui::Text("%s", buffer);
  ImGui::PopFont();

  // speed unit
  ImGui::PushFont(speed_unit_font_);
  ImVec2 textSize_Unit = ImGui::CalcTextSize(" km/h");

  // calculate the position of the text
  double speed_unit_text_x = (70 - textSize_Unit.x) / 2.0f;
  ImGui::SetCursorPos(ImVec2(501 + speed_unit_text_x, 419));
  ImGui::Text(" km/h");
  ImGui::PopFont();

  // imu
  ImGui::GetWindowDrawList()->AddCircle(ImVec2(720, 180), 45, ImColor(0, 0, 0),
                                        0, 2.0f);
  ImGui::GetWindowDrawList()->AddLine(
      ImVec2(720, 180 - 45), ImVec2(720, 180 + 45), ImColor(0, 0, 0), 2.0f);
  ImGui::GetWindowDrawList()->AddLine(
      ImVec2(720 - 45, 180), ImVec2(720 + 45, 180), ImColor(0, 0, 0), 2.0f);
  double x = 45 * data_->imu_acceleration[0] / 2.0f;
  double y = 45 * data_->imu_acceleration[1] / 2.0f;
  ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(720 + x, 180 - y), 3,
                                              ImColor(255, 0, 0));

  // tire temperature color
  ImGui::PushFont(speed_unit_font_);
  get_tire_temperature_color(data_->tire_temperature[0], ImVec2(685, 300),
                             ImVec2(715, 360));
  get_tire_temperature_color(data_->tire_temperature[1], ImVec2(730, 300),
                             ImVec2(760, 360));
  get_tire_temperature_color(data_->tire_temperature[2], ImVec2(685, 380),
                             ImVec2(715, 440));
  get_tire_temperature_color(data_->tire_temperature[3], ImVec2(730, 380),
                             ImVec2(760, 440));
  ImGui::PopFont();

  // tire temperature background
  ImGui::GetWindowDrawList()->AddRect(ImVec2(685, 300), ImVec2(715, 360),
                                      IM_COL32(0, 0, 0, 255), 5.0f, 0, 3.0f);
  ImGui::GetWindowDrawList()->AddRect(ImVec2(730, 300), ImVec2(760, 360),
                                      IM_COL32(0, 0, 0, 255), 5.0f, 0, 3.0f);
  ImGui::GetWindowDrawList()->AddRect(ImVec2(685, 380), ImVec2(715, 440),
                                      IM_COL32(0, 0, 0, 255), 5.0f, 0, 3.0f);
  ImGui::GetWindowDrawList()->AddRect(ImVec2(730, 380), ImVec2(760, 440),
                                      IM_COL32(0, 0, 0, 255), 5.0f, 0, 3.0f);

  ImGui::End();
}

void Screen::display_sensor_data() {
  std::lock_guard<std::mutex> lock(data_->mutex);

  // data font
  ImGui::PushFont(sensor_data_font_);

  /* first column ------------------------------------------------------------*/
  // wheel speed
  ImGui::SetCursorPos(ImVec2(30, 30));
  ImGui::Text("Wheel Speed:");
  ImGui::SetCursorPos(ImVec2(50, 55));
  ImGui::Text("FL: %.2f RPM\nFR: %.2f RPM\nRL: %.2f RPM\nRR: %.2f RPM\n",
              data_->wheel_speed[0], data_->wheel_speed[1],
              data_->wheel_speed[2], data_->wheel_speed[3]);

  // tire tempature
  ImGui::SetCursorPos(ImVec2(30, 125));
  ImGui::Text("Tire Temperature:");
  ImGui::SetCursorPos(ImVec2(50, 150));
  ImGui::Text("FL: %.2f °C \nFR: %.2f °C \nRL: %.2f °C \nRR: %.2f °C \n",
              data_->tire_temperature[0], data_->tire_temperature[1],
              data_->tire_temperature[2], data_->tire_temperature[3]);

  // suspenion travel
  ImGui::SetCursorPos(ImVec2(30, 220));
  ImGui::Text("Suspension Travel:");
  ImGui::SetCursorPos(ImVec2(50, 245));
  ImGui::Text("FL: %.2f mm\nFR: %.2f mm\nRL: %.2f mm\nRR: %.2f mm\n",
              data_->suspension[0], data_->suspension[1], data_->suspension[2],
              data_->suspension[3]);

  // brake oil pressure
  ImGui::SetCursorPos(ImVec2(30, 315));
  ImGui::Text("Brake Oil Pressure:");
  ImGui::SetCursorPos(ImVec2(50, 340));
  ImGui::Text("Front: %.2f bar\nRear: %.2f bar", data_->brake_pressure[0],
              data_->brake_pressure[1]);

  // other temperatures
  ImGui::SetCursorPos(ImVec2(30, 380));
  ImGui::Text("Temperatures:");
  ImGui::SetCursorPos(ImVec2(50, 405));
  ImGui::Text("Motor Temp: %.2f °C \nBattery Temp: %.2f °C",
              data_->motor_temperature, data_->battery_temperature);

  /* second column -----------------------------------------------------------*/
  // steer angle text
  ImGui::SetCursorPos(ImVec2(269, 30));
  ImGui::Text("Steer Angle:");

  // steer angle
  ImGui::PushFont(steer_angle_font_);
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%.0f", data_->steer_angle);

  // calculate the position of the steer angle text
  ImVec2 steer_angle_text_size = ImGui::CalcTextSize(buffer);
  double steer_angle_text_x = (261 - steer_angle_text_size.x) / 2.0f;
  ImGui::SetCursorPosX(269 + steer_angle_text_x);
  ImGui::Text("%s", buffer);
  ImGui::PopFont();

  // imu
  ImGui::SetCursorPos(ImVec2(269, 90));
  ImGui::Text("IMU:");
  ImGui::SetCursorPos(ImVec2(289, 115));
  ImGui::Text("X: %.6f g \nY: %.6f g \nZ: %.6f g", data_->imu_acceleration[0],
              data_->imu_acceleration[1], data_->imu_acceleration[2]);

  ImGui::SetCursorPos(ImVec2(269, 170));
  ImGui::Text("Angular Velocity:");
  ImGui::SetCursorPos(ImVec2(289, 195));
  ImGui::Text("X: %.2f °/s \nY: %.2f °/s \nZ: %.2f °/s",
              data_->imu_angular_velocity[0], data_->imu_angular_velocity[1],
              data_->imu_angular_velocity[2]);

  ImGui::SetCursorPos(ImVec2(269, 250));
  ImGui::Text("Quaternion:");
  ImGui::SetCursorPos(ImVec2(289, 275));
  ImGui::Text("(%.4f, %.4f,\n%.4f, %.4f)", data_->imu_quaternion[0],
              data_->imu_quaternion[1], data_->imu_quaternion[2],
              data_->imu_quaternion[3]);

  // gps
  ImGui::SetCursorPos(ImVec2(269, 315));
  ImGui::Text("GPS:");
  ImGui::SetCursorPos(ImVec2(289, 340));
  ImGui::Text("Latitude: %.5f °N\nLongitude: %.5f °E\nAltitude: %.f m",
              data_->gps_position[0], data_->gps_position[1],
              data_->gps_position[2]);
  ImGui::SetCursorPos(ImVec2(269, 395));
  ImGui::Text("GPS Speed:");
  ImGui::SetCursorPos(ImVec2(289, 420));
  ImGui::Text("X: %.f m/s\nY: %.f m/s\nZ: %.f m/s", data_->gps_velocity[0],
              data_->gps_velocity[1], data_->gps_velocity[2]);

  /* third column ------------------------------------------------------------*/
  // inverter
  ImGui::SetCursorPos(ImVec2(551, 30));
  ImGui::Text("Inverter:");
  ImGui::SetCursorPos(ImVec2(571, 55));
  ImGui::Text(
      "DC Voltage: %.1f V\nControl Board Temp: %.1f °C\nHot Spot Temp: %.1f "
      "°C\nState: %u \nVSM State: %u \n",
      data_->inverter_dc_voltage, data_->inverter_control_board_temperature,
      data_->inverter_hot_spot_temperature, data_->inverter_state,
      data_->inverter_vsm_state);

  // bux status
  ImGui::SetCursorPos(ImVec2(551, 140));
  ImGui::Text("Box Status:");
  ImGui::SetCursorPos(ImVec2(571, 165));
  ImGui::Text("VCU State: %s\nRTD Condition: 0x%X\nRear Box State: %s",
              status_map.at(data_->vcu_state).c_str(), data_->rtd_condition,
              status_map.at(data_->rear_box_state).c_str());

  // rpi stats
  ImGui::SetCursorPos(ImVec2(551, 220));
  ImGui::Text("RPi:");
  ImGui::SetCursorPos(ImVec2(571, 245));
  ImGui::Text(
      "CPU: %.1f %%\nCPU Temp: %.1f °C\nMemory: %.1f %%\nDisk: %.1f %%\nSwap: "
      "%.1f %%",
      100 * data_->cpu_usage, data_->cpu_temperature, 100 * data_->memory_usage,
      100 * data_->disk_usage, 100 * data_->swap_usage);

  // data font
  ImGui::PopFont();

  ImGui::End();
}

/* static function definitions -----------------------------------------------*/
#ifdef BACKEND_GLFW
static void glfw_error_callback(int error, const char *description) {
  std::cerr << "GLFW Error " << description << ": " << error << std::endl;
}
#endif
