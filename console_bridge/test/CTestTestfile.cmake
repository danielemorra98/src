# CMake generated Testfile for 
# Source directory: /home/lorenzo/mbzirc2020_ws/src/console_bridge/test
# Build directory: /home/lorenzo/mbzirc2020_ws/src/console_bridge/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(console_TEST "/home/lorenzo/mbzirc2020_ws/src/console_bridge/bin/console_TEST" "--gtest_output=xml:/home/lorenzo/mbzirc2020_ws/src/console_bridge/test_results/console_TEST.xml")
set_tests_properties(console_TEST PROPERTIES  TIMEOUT "240")
