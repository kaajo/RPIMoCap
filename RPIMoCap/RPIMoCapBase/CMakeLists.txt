cmake_minimum_required(VERSION 3.1)

project(RPIMoCapBase CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -O3")

find_package(Qt5 COMPONENTS Core REQUIRED)
find_library(mosquitto REQUIRED)
find_library(mosquittopp REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

QT5_WRAP_CPP(QOBJECT_HEADERS
    include/RPIMoCapBase/mqttpublisher.h
    include/RPIMoCapBase/mqttsubscriber.h)

set(SOURCES
    ${QOBJECT_HEADERS}
    src/frame.cpp
    src/line3d.cpp
    src/mqttpublisher.cpp
    src/mqttsettings.cpp
    src/mqttsubscriber.cpp)

add_library(RPIMoCapBase STATIC ${SOURCES})

target_include_directories(RPIMoCapBase PUBLIC
    ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(RPIMoCapBase
    mosquitto
    mosquittopp
    Qt5::Core)
