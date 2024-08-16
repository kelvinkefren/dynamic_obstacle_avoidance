# CMake generated Testfile for 
# Source directory: /home/kelvin/master_ws_newer/src/dynamic_obstacle_avoidance
# Build directory: /home/kelvin/master_ws_newer/src/dynamic_obstacle_avoidance/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_dynamic_obstacle_avoidance_nosetests_test.test_scenario_controller.py "/home/kelvin/master_ws_newer/src/dynamic_obstacle_avoidance/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/kelvin/master_ws_newer/src/dynamic_obstacle_avoidance/build/test_results/dynamic_obstacle_avoidance/nosetests-test.test_scenario_controller.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/kelvin/master_ws_newer/src/dynamic_obstacle_avoidance/build/test_results/dynamic_obstacle_avoidance" "/usr/bin/nosetests3 -P --process-timeout=60 /home/kelvin/master_ws_newer/src/dynamic_obstacle_avoidance/test/test_scenario_controller.py --with-xunit --xunit-file=/home/kelvin/master_ws_newer/src/dynamic_obstacle_avoidance/build/test_results/dynamic_obstacle_avoidance/nosetests-test.test_scenario_controller.py.xml")
set_tests_properties(_ctest_dynamic_obstacle_avoidance_nosetests_test.test_scenario_controller.py PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/kelvin/master_ws_newer/src/dynamic_obstacle_avoidance/CMakeLists.txt;71;catkin_add_nosetests;/home/kelvin/master_ws_newer/src/dynamic_obstacle_avoidance/CMakeLists.txt;0;")
subdirs("gtest")
