cmake_minimum_required(VERSION 3.20)
project(maze_viewer LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# the engine (git submodule): window, GL context, shader and camera classes
add_subdirectory(external/opengl-quickstart)

add_executable(maze_viewer src/main.cpp)
target_link_libraries(maze_viewer PRIVATE opengl_quickstart)
target_compile_definitions(maze_viewer PRIVATE MV_SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders/")
quickstart_setup_executable(maze_viewer)
