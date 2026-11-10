# The optional control toolkit: MPC, a per-step LP and NMPC. It links the optimization toolkit's
# Workspace. In-tree, the engine adds the optimization package first (or this file adds it); the
# target check keeps the engine's own optional-toolkit loop from adding it twice.
if(NOT TARGET paperboat_toolkit_optimization)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../optimization/CMakeLists.txt")
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../optimization" "${CMAKE_BINARY_DIR}/optimization-for-control")
  else()
    message(FATAL_ERROR "The control toolkit needs the optimization toolkit (install optimization first)")
  endif()
endif()

add_library(paperboat_toolkit_mpc src/blocks.cpp)
add_library(paperboat::toolkit_mpc ALIAS paperboat_toolkit_mpc)
target_include_directories(paperboat_toolkit_mpc PUBLIC include)
target_link_libraries(paperboat_toolkit_mpc
  PUBLIC paperboat::toolkit
  PRIVATE paperboat_toolkit_optimization paperboat_warnings)

if(PAPERBOAT_BUILD_TESTS)
  add_executable(paperboat_mpc_tests tests/mpc_test.cpp)
  target_include_directories(paperboat_mpc_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(paperboat_mpc_tests PRIVATE paperboat_toolkit_mpc Catch2::Catch2WithMain)
  add_test(NAME paperboat_mpc_tests COMMAND paperboat_mpc_tests)
endif()
