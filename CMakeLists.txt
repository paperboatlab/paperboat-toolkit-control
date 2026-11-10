# paperboat-toolkit-control: the optional control toolkit for Paperboat.
#
# Builds paperboat::toolkit_mpc. It links the optimization toolkit's Workspace.
#   1. In-tree: copy this folder into a paperboat checkout at engine/toolkit/mpc and
#      install the optimization toolkit at engine/toolkit/optimization; the engine discovers
#      both from their toolkit.toml.
#   2. Standalone: set PAPERBOAT_SOURCE_DIR and PAPERBOAT_OPTIMIZATION_DIR, or let this file
#      fetch paperboat and the optimization toolkit from GitHub.

cmake_minimum_required(VERSION 3.20)
project(paperboat_toolkit_mpc LANGUAGES CXX)

include(FetchContent)

if(NOT DEFINED PAPERBOAT_SOURCE_DIR OR NOT EXISTS "${PAPERBOAT_SOURCE_DIR}/engine/CMakeLists.txt")
  message(STATUS "PAPERBOAT_SOURCE_DIR not set; fetching paperboat engine from GitHub")
  FetchContent_Declare(paperboat
    GIT_REPOSITORY https://github.com/paperboatlab/paperboat.git
    GIT_TAG dev)
  FetchContent_MakeAvailable(paperboat)
  set(PAPERBOAT_SOURCE_DIR "${paperboat_SOURCE_DIR}" CACHE PATH "Path to the paperboat source tree")
endif()

if(NOT DEFINED PAPERBOAT_OPTIMIZATION_DIR OR NOT EXISTS "${PAPERBOAT_OPTIMIZATION_DIR}/CMakeLists.txt")
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../paperboat-toolkit-optimization/engine/toolkit/optimization/CMakeLists.txt")
    set(PAPERBOAT_OPTIMIZATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../paperboat-toolkit-optimization/engine/toolkit/optimization")
  else()
    message(STATUS "PAPERBOAT_OPTIMIZATION_DIR not set; fetching the optimization toolkit from GitHub")
    FetchContent_Declare(optimization
      GIT_REPOSITORY https://github.com/paperboatlab/paperboat-toolkit-optimization.git
      GIT_TAG dev)
    FetchContent_MakeAvailable(optimization)
    set(PAPERBOAT_OPTIMIZATION_DIR "${optimization_SOURCE_DIR}/engine/toolkit/optimization")
  endif()
endif()

add_subdirectory("${PAPERBOAT_SOURCE_DIR}/engine" "${CMAKE_BINARY_DIR}/paperboat-engine" EXCLUDE_FROM_ALL)
add_subdirectory("${PAPERBOAT_OPTIMIZATION_DIR}" "${CMAKE_BINARY_DIR}/optimization")
add_subdirectory(engine/toolkit/mpc)
