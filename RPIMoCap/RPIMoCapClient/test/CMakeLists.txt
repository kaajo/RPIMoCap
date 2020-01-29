cmake_minimum_required(VERSION 3.13)
project(RPIMoCapClientTest CXX)

enable_testing()
find_package(GTest REQUIRED)

add_executable(testSimScene ../src/SimClient/simscene.cpp simscene.cpp)
target_link_libraries(testSimScene RPIMoCapClientLib GTest::GTest GTest::Main)
gtest_discover_tests(testSimScene)
