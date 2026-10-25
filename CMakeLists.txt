cmake_minimum_required(VERSION 3.24)
project(procedural-motion VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
  GIT_SHALLOW TRUE)
FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG 1.0.1
  GIT_SHALLOW TRUE)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
  GIT_SHALLOW TRUE)
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.8
  GIT_SHALLOW TRUE)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(glfw glm nlohmann_json)

# ImGui does not ship a CMake project. Populate the sources and compile
# only the files this lab uses.
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
  FetchContent_Populate(imgui)
endif()

add_library(proc_core STATIC
  src/body/skeleton.cpp
  src/body/proc_body.cpp
  src/body/joint_limits.cpp
  src/motion/proc_motion.cpp
  src/motion/proc_motion_load.cpp
  src/motion/interpolation.cpp
  src/motion/motion_player.cpp
  src/rig/forward_kinematics.cpp
  src/validation/skeleton_validator.cpp
  src/validation/motion_validator.cpp
  src/validation/bone_length_monitor.cpp)
target_include_directories(proc_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(proc_core PUBLIC glm::glm nlohmann_json::nlohmann_json)
if(MSVC)
  target_compile_options(proc_core PRIVATE /W4 /permissive-)
  target_compile_definitions(proc_core PRIVATE NOMINMAX)
endif()

add_library(imgui_lib STATIC
  "${imgui_SOURCE_DIR}/imgui.cpp"
  "${imgui_SOURCE_DIR}/imgui_draw.cpp"
  "${imgui_SOURCE_DIR}/imgui_tables.cpp"
  "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
  "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
  "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp")
target_include_directories(imgui_lib PUBLIC
  "${imgui_SOURCE_DIR}"
  "${imgui_SOURCE_DIR}/backends")
target_link_libraries(imgui_lib PUBLIC glfw)

add_executable(proc_motion_lab
  src/main.cpp
  src/app/application.cpp
  src/render/renderer.cpp
  src/render/skeleton_renderer.cpp)
target_link_libraries(proc_motion_lab PRIVATE proc_core imgui_lib glfw opengl32 comdlg32)
if(MSVC)
  target_compile_options(proc_motion_lab PRIVATE /W4 /permissive-)
  target_compile_definitions(proc_motion_lab PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

enable_testing()
file(TO_CMAKE_PATH "${CMAKE_SOURCE_DIR}" PROCMOTION_SOURCE_DIR_FWD)
add_executable(proc_motion_tests
  tests/test_main.cpp
  tests/body_tests.cpp
  tests/fk_tests.cpp
  tests/motion_tests.cpp)
target_link_libraries(proc_motion_tests PRIVATE proc_core)
target_compile_definitions(proc_motion_tests PRIVATE PROCMOTION_SOURCE_DIR="${PROCMOTION_SOURCE_DIR_FWD}")
if(MSVC)
  target_compile_options(proc_motion_tests PRIVATE /W4 /permissive-)
  target_compile_definitions(proc_motion_tests PRIVATE NOMINMAX)
endif()
add_test(NAME proc_motion_tests COMMAND proc_motion_tests)
