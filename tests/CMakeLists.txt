cmake_minimum_required(VERSION 3.22)
project(line_follow_tests C)
enable_testing()
add_executable(test_controller test_controller.c ../Control/line_follower.c ../Control/bridge_logic.c)
target_include_directories(test_controller PRIVATE ../Control)
target_compile_options(test_controller PRIVATE -std=c11 -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(test_controller PRIVATE m)
foreach(case calibration centroid lost reacquire timeout stop driver stale timing config direction stall pid bridge rollover allblack weakline integral extrapolation scurve obstacle probe probeaim
              probefit probeside probepar probeleg probeturn probetour
              motorcal ground_priority hwemu hwemu_netlist)
  add_test(NAME ${case} COMMAND test_controller ${case})
endforeach()
