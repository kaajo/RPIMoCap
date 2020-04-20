cmake_minimum_required(VERSION 3.13)
project(RPIMoCapServerTest CXX)

enable_testing()
find_package(GTest REQUIRED)

add_executable(testWandDetector wanddetector.cpp)
target_link_libraries(testWandDetector PRIVATE RPIMoCapServerLib ${OPENCV_LIBRARIES} GTest::GTest GTest::Main)
gtest_discover_tests(testWandDetector)

add_executable(testWandCalibration wandcalibration.cpp)
target_link_libraries(testWandCalibration PRIVATE RPIMoCapServerLib ${OPENCV_LIBRARIES} GTest::GTest GTest::Main)
gtest_discover_tests(testWandCalibration)
