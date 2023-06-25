# set imgui backend to use
if(NOT DEFINED IMGUI_BACKEND)
    set(IMGUI_BACKEND GLFW CACHE STRING "ImGui backend to use")
endif()

if(IMGUI_BACKEND STREQUAL "GLFW")
        add_compile_definitions(BACKEND_GLFW)
elseif(IMGUI_BACKEND STREQUAL "SDL")
        add_compile_definitions(BACKEND_SDL)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find apple specific graphics library
if(APPLE)
        find_library(OPENGL_LIBRARY OpenGL REQUIRED)
        find_library(COCOA_LIBRARY Cocoa REQUIRED)
        find_library(IOKIT_LIBRARY IOKit REQUIRED)
        find_library(COREVID_LIBRARY CoreVideo REQUIRED)
endif()

# imgui includes
include_directories(
        imgui
        imgui/backends
)

# pull apple global includes
if(APPLE)
        include_directories(
                /usr/local/include
                /opt/local/include
                /opt/homebrew/include
        )
endif()

# pull sdl2 includes
if(IMGUI_BACKEND STREQUAL "SDL")
        include_directories(
                /usr/include/SDL2
        )
endif()

# set common imgui sources
file(GLOB IMGUI_SRC
        imgui/*.h
        imgui/*.cpp
)

# set platform specific imgui sources
if(WIN32)
        file(GLOB IMGUI_PLATFORM_SRC
                imgui/backends/imgui_impl_win32.*
                imgui/backends/imgui_impl_dx12.*
        )
elseif(UNIX)
        if(IMGUI_BACKEND STREQUAL "GLFW")
                add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)
                include_directories(
                        imgui/examples/libs/gl3w
                )
                file(GLOB IMGUI_BACKEND_SRC
                        imgui/examples/libs/gl3w/GL/gl3w.*
                        imgui/backends/imgui_impl_glfw.*
                )
        elseif(IMGUI_BACKEND STREQUAL "SDL")
                file(GLOB IMGUI_BACKEND_SRC
                        imgui/backends/imgui_impl_sdl2.*
                )
        endif()
        file(GLOB IMGUI_PLATFORM_SRC
                imgui/backends/imgui_impl_opengl2.*
        )
endif()

# link apple globle libraries
if(APPLE)
        link_directories(
                /usr/local/lib
                /opt/local/lib
                /opt/homebrew/lib
        )
endif()

# library: imgui
add_library(imgui SHARED
        ${IMGUI_SRC}
        ${IMGUI_BACKEND_SRC}
        ${IMGUI_PLATFORM_SRC}
)

# link to platform specific libraries
if(WIN32)
        target_link_libraries(imgui
                d3d12.lib
                d3compiler.lib
                dxgi.lib
        )
elseif(APPLE)
        target_link_libraries(imgui
                ${OPENGL_LIBRARY}
                ${COCOA_LIBRARY}
                ${IOKIT_LIBRARY}
                ${COREVID_LIBRARY}
        )
        if(IMGUI_BACKEND STREQUAL "GLFW")
                target_link_libraries(imgui
                        glfw
                )
        elseif(IMGUI_BACKEND STREQUAL "SDL")
                target_link_libraries(imgui
                        SDL2
                )
        endif()
elseif(UNIX AND NOT APPLE)
        target_link_libraries(imgui
                GL
                dl
        )
        if(IMGUI_BACKEND STREQUAL "GLFW")
                target_link_libraries(imgui
                        glfw
                )
        elseif(IMGUI_BACKEND STREQUAL "SDL")
                target_link_libraries(imgui
                        SDL2
                )
        endif()
endif()
