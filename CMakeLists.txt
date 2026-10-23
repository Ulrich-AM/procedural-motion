cmake_minimum_required(VERSION 3.24)

# Assimp 5.4 and other fetched projects still declare older CMake minimums.
if(POLICY CMP0000)
  cmake_policy(SET CMP0000 NEW)
endif()
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

project(procedural_motion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/MP)
endif()

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# MuJoCo ships a prebuilt Windows binary. Building it from source is unnecessary.
set(PM_MUJOCO_VERSION 3.17.0)
set(PM_MUJOCO_ZIP "${CMAKE_BINARY_DIR}/_deps/mujoco-${PM_MUJOCO_VERSION}-windows-x86_64.zip")
set(PM_MUJOCO_EXTRACT "${CMAKE_BINARY_DIR}/_deps/mujoco-extract")
if(NOT EXISTS "${PM_MUJOCO_ZIP}")
  message(STATUS "Downloading MuJoCo ${PM_MUJOCO_VERSION}")
  file(DOWNLOAD
    "https://github.com/google-deepmind/mujoco/releases/download/${PM_MUJOCO_VERSION}/mujoco-${PM_MUJOCO_VERSION}-windows-x86_64.zip"
    "${PM_MUJOCO_ZIP}"
    SHOW_PROGRESS
    TIMEOUT 600
    STATUS PM_MUJOCO_STATUS)
  list(GET PM_MUJOCO_STATUS 0 PM_MUJOCO_CODE)
  if(NOT PM_MUJOCO_CODE EQUAL 0)
    list(GET PM_MUJOCO_STATUS 1 PM_MUJOCO_MESSAGE)
    message(FATAL_ERROR "MuJoCo download failed: ${PM_MUJOCO_MESSAGE}")
  endif()
endif()
if(NOT EXISTS "${PM_MUJOCO_EXTRACT}")
  file(ARCHIVE_EXTRACT INPUT "${PM_MUJOCO_ZIP}" DESTINATION "${PM_MUJOCO_EXTRACT}")
endif()

function(pm_find_root out_var search_dir marker)
  if(EXISTS "${search_dir}/${marker}")
    set(${out_var} "${search_dir}" PARENT_SCOPE)
    return()
  endif()
  file(GLOB children LIST_DIRECTORIES true "${search_dir}/*")
  foreach(child ${children})
    if(IS_DIRECTORY "${child}" AND EXISTS "${child}/${marker}")
      set(${out_var} "${child}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
  set(${out_var} "" PARENT_SCOPE)
endfunction()

pm_find_root(MUJOCO_ROOT "${PM_MUJOCO_EXTRACT}" "include/mujoco/mujoco.h")
if(NOT MUJOCO_ROOT)
  message(FATAL_ERROR "MuJoCo headers were not found after extracting ${PM_MUJOCO_ZIP}")
endif()
if(EXISTS "${MUJOCO_ROOT}/lib/mujoco.lib")
  set(MUJOCO_IMPLIB "${MUJOCO_ROOT}/lib/mujoco.lib")
else()
  set(MUJOCO_IMPLIB "${MUJOCO_ROOT}/bin/mujoco.lib")
endif()
if(NOT EXISTS "${MUJOCO_ROOT}/bin/mujoco.dll" OR NOT EXISTS "${MUJOCO_IMPLIB}")
  message(FATAL_ERROR "MuJoCo import library or DLL is missing under ${MUJOCO_ROOT}")
endif()

add_library(mujoco SHARED IMPORTED GLOBAL)
set_target_properties(mujoco PROPERTIES
  IMPORTED_LOCATION "${MUJOCO_ROOT}/bin/mujoco.dll"
  IMPORTED_IMPLIB "${MUJOCO_IMPLIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_ROOT}/include")

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  glfw
  URL https://github.com/glfw/glfw/archive/refs/tags/3.4.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(glfw)

set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(nlohmann_json)

set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)
set(ASSIMP_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ALL_EXPORTERS_BY_DEFAULT OFF CACHE BOOL "" FORCE)
set(ASSIMP_NO_EXPORT ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_FBX_IMPORTER ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ZLIB ON CACHE BOOL "" FORCE)
FetchContent_Declare(
  assimp
  URL https://github.com/assimp/assimp/archive/refs/tags/v5.4.3.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(assimp)

FetchContent_Declare(
  imgui
  URL https://github.com/ocornut/imgui/archive/refs/tags/v1.91.8.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
  FetchContent_Populate(imgui)
endif()
pm_find_root(IMGUI_DIR "${imgui_SOURCE_DIR}" "imgui.h")
if(NOT IMGUI_DIR)
  message(FATAL_ERROR "Dear ImGui sources were not found")
endif()

add_library(imgui STATIC
  "${IMGUI_DIR}/imgui.cpp"
  "${IMGUI_DIR}/imgui_draw.cpp"
  "${IMGUI_DIR}/imgui_tables.cpp"
  "${IMGUI_DIR}/imgui_widgets.cpp"
  "${IMGUI_DIR}/backends/imgui_impl_glfw.cpp"
  "${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp")
target_include_directories(imgui PUBLIC "${IMGUI_DIR}" "${IMGUI_DIR}/backends")
target_link_libraries(imgui PUBLIC glfw)
if(WIN32)
  target_link_libraries(imgui PUBLIC opengl32)
endif()

set(PM_FBX "${CMAKE_SOURCE_DIR}/assets/source/ybot.fbx")
if(NOT EXISTS "${PM_FBX}")
  message(FATAL_ERROR "Missing ${PM_FBX}")
endif()

add_executable(fbx_skeleton_import tools/fbx_skeleton_import/main.cpp)
target_include_directories(fbx_skeleton_import PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(fbx_skeleton_import PRIVATE assimp nlohmann_json::nlohmann_json)
if(MSVC)
  target_compile_options(fbx_skeleton_import PRIVATE /W3)
endif()

set(PM_SKELETON_JSON "${CMAKE_SOURCE_DIR}/assets/generated/skeleton.json")
add_custom_command(
  OUTPUT "${PM_SKELETON_JSON}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_SOURCE_DIR}/assets/generated"
  COMMAND "$<TARGET_FILE:fbx_skeleton_import>" "${PM_FBX}" "${PM_SKELETON_JSON}"
  DEPENDS fbx_skeleton_import "${PM_FBX}"
  COMMENT "Importing Y-Bot skeleton (mesh and animation clips are ignored)"
  VERBATIM)
add_custom_target(generate_skeleton DEPENDS "${PM_SKELETON_JSON}")

add_executable(procedural_motion
  src/main.cpp
  src/app/application.cpp
  src/app/sim_config.cpp
  src/physics/physical_skeleton.cpp
  src/control/standing_controller.cpp
  src/render/viewer.cpp
  src/skeleton/skeleton_definition.cpp)
add_dependencies(procedural_motion generate_skeleton)
target_include_directories(procedural_motion PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(procedural_motion PRIVATE mujoco glfw imgui nlohmann_json::nlohmann_json)
target_compile_definitions(procedural_motion PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
if(MSVC)
  target_compile_options(procedural_motion PRIVATE /W3)
endif()

add_custom_command(TARGET procedural_motion POST_BUILD
  COMMAND "${CMAKE_COMMAND}" -E copy_directory "${CMAKE_SOURCE_DIR}/assets" "$<TARGET_FILE_DIR:procedural_motion>/assets"
  COMMAND "${CMAKE_COMMAND}" -E copy_directory "${CMAKE_SOURCE_DIR}/config" "$<TARGET_FILE_DIR:procedural_motion>/config"
  COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${MUJOCO_ROOT}/bin/mujoco.dll" "$<TARGET_FILE_DIR:procedural_motion>"
  VERBATIM)

enable_testing()
add_test(NAME phase0_physics COMMAND $<TARGET_FILE:procedural_motion> --self-test)

file(GLOB PM_MUJOCO_EXTRA_DLLS "${MUJOCO_ROOT}/bin/*.dll")
foreach(extra_dll ${PM_MUJOCO_EXTRA_DLLS})
  add_custom_command(TARGET procedural_motion POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${extra_dll}" "$<TARGET_FILE_DIR:procedural_motion>"
    VERBATIM)
endforeach()
