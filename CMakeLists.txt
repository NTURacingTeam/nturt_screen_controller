cmake_minimum_required(VERSION 3.8)
project(nturt_screen_controller)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

set(IMGUI_BACKEND SDL CACHE STRING "ImGui backend to use")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(nturt_can_config REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# configure imgui
include(cmake/imgui.cmake)

include_directories(
        include
)

# libraries
# library: nturt_screen_controller
add_library(${PROJECT_NAME} SHARED
        src/screen_controller.cpp
        src/screen.cpp
)
target_link_libraries(${PROJECT_NAME}
        imgui
)
ament_target_dependencies(${PROJECT_NAME}
        ament_index_cpp
        can_msgs
        nturt_can_config
        rclcpp
        rclcpp_components
)
# register to node components
# node component: ScreenController
rclcpp_components_register_nodes(nturt_screen_controller "ScreenController")
set(node_plugins "${node_plugins}ScreenController;$<TARGET_FILE:nturt_screen_controller>\n")

# applications
# executable: nturt_screen_controller_node
add_executable(${PROJECT_NAME}_node
        app/screen_controller_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
)

# test: screen_test_node
add_executable(screen_test_node
        test/screen_test.cpp
)
target_link_libraries(screen_test_node
        ${PROJECT_NAME}
)
ament_target_dependencies(screen_test_node
        rclcpp
)

# install headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

# install libraries
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

# install executables
install(TARGETS ${PROJECT_NAME}_node screen_test_node
        DESTINATION lib/${PROJECT_NAME}
)

# install fonts
install(DIRECTORY fonts
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # the following line skips the linter which checks for copyrights
        # comment the line when a copyright and license is added to all source files
        set(ament_cmake_copyright_FOUND TRUE)
        # the following line skips cpplint (only works in a git repo)
        # comment the line when this package is in a git repo and when
        # a copyright and license is added to all source files
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
endif()

# export package dependencies
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        ament_cmake
        ament_index_cpp
        can_msgs
        nturt_can_config
        rclcpp
        rclcpp_components
)

ament_package()
