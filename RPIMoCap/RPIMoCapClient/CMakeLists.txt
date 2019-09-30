cmake_minimum_required(VERSION 3.1)

project(RPIMoCapClient CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -O3")

find_library(mosquitto REQUIRED)
find_library(mosquittopp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets Network Concurrent REQUIRED)
pkg_check_modules(MSGPACK REQUIRED msgpack)
pkg_check_modules(OPENCV REQUIRED opencv)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

QT5_WRAP_CPP(QOBJECT_HEADERS
    include/RPIMoCap/ClientLib/rpimocapclient.h)

set(SOURCES
    src/ClientLib/markerdetector.cpp
    src/ClientLib/rpicamera.cpp
    src/ClientLib/rpimocapclient.cpp)

add_library(RPIMoCapClientLib STATIC ${QOBJECT_HEADERS} ${SOURCES})

target_include_directories(RPIMoCapClientLib PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${GSTREAMER_BASE_INCLUDE_DIRS})

target_link_libraries(RPIMoCapClientLib
    RPIMoCapBase
    ${MSGPACK_LIBRARIES}
    ${OPENCV_LIBRARIES}
    ${GSTREAMER_BASE_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    Qt5::Core
    Qt5::Gui
    Qt5::Network
    Qt5::Concurrent)

add_executable(RPIMoCapClient src/Client/main.cpp)
target_link_libraries(RPIMoCapClient RPIMoCapClientLib)

QT5_WRAP_CPP(QOBJECT_SIM_HEADERS
    include/RPIMoCap/SimClient/simcamerawidget.h
    include/RPIMoCap/SimClient/mainwindow.h)

add_executable(RPIMoCapSimClient
    ${QOBJECT_SIM_HEADERS}
    src/SimClient/main.cpp
    src/SimClient/simcamera.cpp
    src/SimClient/simcamerawidget.cpp
    src/SimClient/simscene.cpp
    src/SimClient/mainwindow.cpp)

target_link_libraries(RPIMoCapSimClient
    RPIMoCapClientLib
    Qt5::Widgets)
