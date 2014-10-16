## Unit tests, built when CATKIN_ENABLE_TESTING is true.
#
catkin_add_gtest(test_plan_concurrently test_plan_concurrently.cpp)
target_link_libraries(test_plan_concurrently ${catkin_LIBRARIES})
