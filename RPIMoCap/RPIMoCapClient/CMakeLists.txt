cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -O3")

find_library(mosquitto REQUIRED)
find_library(mosquittopp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Network Concurrent REQUIRED)
pkg_check_modules(MSGPACK REQUIRED msgpack)
pkg_check_modules(OPENCV REQUIRED opencv)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(SOURCES
    avahibrowser.cpp
    markerdetector.cpp
    rpicamera.cpp
    rpimocapclient.cpp
    main.cpp)

add_executable(RPIMoCapClient ${SOURCES})

target_include_directories(RPIMoCapClient PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${GSTREAMER_BASE_INCLUDE_DIRS})

target_link_libraries(RPIMoCapClient
    RPIMoCapBase
    ${MSGPACK_LIBRARIES}
    ${OPENCV_LIBRARIES}
    ${GSTREAMER_BASE_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    Qt5::Core
    Qt5::Gui
    Qt5::Network
    Qt5::Concurrent)
