# The optional control toolkit: the clock dynamics and per-step MPC, LP and NMPC. It links
# the optimization toolkit's Workspace. In-tree, the engine adds the optimization package first
# (or this file adds it); the target check keeps the engine's own optional-toolkit loop from
# adding it twice.
if(NOT TARGET paperboat_toolkit_optimization)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../optimization/CMakeLists.txt")
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../optimization" "${CMAKE_BINARY_DIR}/optimization-for-control")
  else()
    message(FATAL_ERROR "The control toolkit needs the optimization toolkit (install optimization first)")
  endif()
endif()

# One folder per block: src/control/<block>/<block>.cpp.
file(GLOB PAPERBOAT_CONTROL_BLOCKS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/control/*/*.cpp")
add_library(paperboat_toolkit_control src/register.cpp ${PAPERBOAT_CONTROL_BLOCKS})
add_library(paperboat::toolkit_control ALIAS paperboat_toolkit_control)
target_include_directories(paperboat_toolkit_control PUBLIC include PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(paperboat_toolkit_control
  PUBLIC paperboat::toolkit
  PRIVATE paperboat_toolkit_optimization paperboat_warnings)

if(PAPERBOAT_BUILD_TESTS)
  add_executable(paperboat_control_tests tests/mpc_test.cpp tests/control_test.cpp tests/frequency_test.cpp tests/lti_test.cpp tests/linear_test.cpp tests/nonlinear_test.cpp)
  target_include_directories(paperboat_control_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(paperboat_control_tests PRIVATE paperboat_toolkit_control Catch2::Catch2WithMain)
  add_test(NAME paperboat_control_tests COMMAND paperboat_control_tests)
endif()
