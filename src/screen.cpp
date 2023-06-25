#include "nturt_screen_controller/screen.hpp"

// stl include
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>

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
  speed_font_ =
      io.Fonts->AddFontFromFileTTF("../fonts/HelveticaNeue.ttc", 160.0f);
  speed_unit_font_ =
      io.Fonts->AddFontFromFileTTF("../fonts/Cousine-Regular.ttf", 25.0f);
  battery_font_ =
      io.Fonts->AddFontFromFileTTF("../fonts/Cousine-Regular.ttf", 50.0f);
  sensor_data_font_ =
      io.Fonts->AddFontFromFileTTF("../fonts/Cousine-Regular.ttf", 15.0f);
  steer_angle_font_ =
      io.Fonts->AddFontFromFileTTF("../fonts/Karla-Regular.ttf", 30.0f);

  return 0;
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
                                              IM_COL32(68, 144, 196, 75));

    // draw screen
    if (!data_->checkmode) {
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
  std::lock_guard<std::mutex> lock(data_->data_mutex);

  // warning
  if (data_->error_code[0] != 0) {
    ImVec2 p = ImGui::GetWindowPos();
    ImVec2 a = ImVec2(p.x + 25, p.y + 25);
    ImVec2 b = ImVec2(p.x + 85, p.y + 85);
    ImVec2 c = ImVec2(p.x + 85, p.y + 25);
    ImVec2 d = ImVec2(p.x + 25, p.y + 85);
    ImGui::GetWindowDrawList()->AddLine(a, b, IM_COL32(255, 0, 0, 255), 15.0f);
    ImGui::GetWindowDrawList()->AddLine(c, d, IM_COL32(255, 0, 0, 255), 15.0f);
  }

  // error
  if (data_->error_code[0] != 0) {
    ImVec2 a = ImVec2(111 + 4, 91);
    ImVec2 b = ImVec2(189 - 4, 91);
    ImVec2 c = ImVec2(150, 25);
    ImGui::GetWindowDrawList()->AddTriangleFilled(a, b, c,
                                                  IM_COL32(255, 255, 0, 255));
  }

  // battery life
  if (data_->battery_life > 75) {
    // green battery
    ImGui::GetWindowDrawList()->AddRectFilled(
        ImVec2(762, 86), ImVec2(762 - data_->battery_life * 1.52, 28),
        IM_COL32(0, 255, 0, 255));
  } else if (data_->battery_life > 50) {
    // yellow battery
    ImGui::GetWindowDrawList()->AddRectFilled(
        ImVec2(762, 28), ImVec2(762 - data_->battery_life * 1.52, 86),
        IM_COL32(255, 255, 0, 255));
  } else if (data_->battery_life > 25) {
    // orange battery
    ImGui::GetWindowDrawList()->AddRectFilled(
        ImVec2(762, 28), ImVec2(762 - data_->battery_life * 1.52, 86),
        IM_COL32(255, 165, 0, 255));
  } else {
    // red battery
    ImGui::GetWindowDrawList()->AddRectFilled(
        ImVec2(762, 28), ImVec2(762 - data_->battery_life * 1.52, 86),
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
  ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
  ImGui::Text("%.0f%%", data_->battery_life);
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
             ImGui::GetWindowPos().y + 217 - data_->brake + 240),
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
             ImGui::GetWindowPos().y + 217 - data_->accelerator + 240),
      ImVec2(ImGui::GetWindowPos().x + 35 + 30 + 50,
             ImGui::GetWindowPos().y + 217 + 240),
      IM_COL32(0, 176, 240, 255));

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
    temp += 0.01f;
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

  ImGui::End();
}

void Screen::display_sensor_data() {
  std::lock_guard<std::mutex> lock(data_->data_mutex);

  // steer angle
  ImGui::PushFont(steer_angle_font_);
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%.2f", data_->steer_angle);

  // calculate the position of the text
  ImVec2 steer_angle_text_size = ImGui::CalcTextSize(buffer);
  double steer_angle_text_x = (261 - steer_angle_text_size.x) / 2.0f;
  ImGui::SetCursorPosX(269 + steer_angle_text_x);
  ImGui::Text("%s", buffer);
  ImGui::PopFont();

  // data font
  ImGui::PushFont(sensor_data_font_);

  // steer angle text
  ImGui::SetCursorPos(ImVec2(269, 36));
  ImGui::Text("Steer Angle: ");

  // wheel speed
  ImGui::SetCursorPos(ImVec2(30, 29));
  ImGui::Text("Wheel Speed: ");
  ImGui::SetCursorPos(ImVec2(30, 54));
  ImGui::Text("FL: %.2f \nFR: %.2f \nRL: %.2f\nRR: %.2f\n",
              data_->wheel_speed[0], data_->wheel_speed[1],
              data_->wheel_speed[2], data_->wheel_speed[3]);

  // tire tempature
  ImGui::SetCursorPos(ImVec2(30, 183));
  ImGui::Text("Wheel Temperature: ");
  ImGui::SetCursorPos(ImVec2(30, 208));
  ImGui::Text("FL: %.2f \nFR: %.2f \nRL: %.2f\nRR: %.2f\n",
              data_->tire_temperature[0], data_->tire_temperature[1],
              data_->tire_temperature[2], data_->tire_temperature[3]);

  // suspenion travel
  ImGui::SetCursorPos(ImVec2(30, 336));
  ImGui::Text("Suspension Travel: ");
  ImGui::SetCursorPos(ImVec2(30, 361));
  ImGui::Text("FL: %.2f \nFR: %.2f \nRL: %.2f\nRR: %.2f\n",
              data_->suspension_travel[0], data_->suspension_travel[1],
              data_->suspension_travel[2], data_->suspension_travel[3]);

  // brake oil pressure
  ImGui::SetCursorPos(ImVec2(571, 55));
  ImGui::Text("Brake Oil Pressure: ");
  ImGui::SetCursorPos(ImVec2(571, 80));
  ImGui::Text("Front: %.2f \nRear: %.2f", data_->brake_oil_pressure[0],
              data_->brake_oil_pressure[1]);

  // gps
  ImGui::SetCursorPos(ImVec2(269, 240));
  ImGui::Text("GPS: ");
  ImGui::SetCursorPos(ImVec2(269, 265));
  ImGui::Text("Latitude: %.2f \nLongitude: %.2f \nAltitude: %.2f",
              data_->gps_position[0], data_->gps_position[1],
              data_->gps_position[2]);

  // imu
  ImGui::SetCursorPos(ImVec2(571, 221));
  ImGui::Text("IMU: ");
  ImGui::SetCursorPos(ImVec2(571, 246));
  ImGui::Text("X: %.2f \nY: %.2f \nZ: %.2f", data_->imu_acceleration[0],
              data_->imu_acceleration[1], data_->imu_acceleration[2]);
  ImGui::PopFont();

  ImGui::End();
}

/* static function definitions -----------------------------------------------*/
#ifdef BACKEND_GLFW
static void glfw_error_callback(int error, const char *description) {
  std::cerr << "GLFW Error " << description << ": " << error << std::endl;
}
#endif
