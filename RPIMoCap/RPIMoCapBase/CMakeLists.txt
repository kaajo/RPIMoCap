cmake_minimum_required(VERSION 3.13)

project(RPIMoCapBase CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -O3")

find_package(Qt5 COMPONENTS Core Network REQUIRED)
find_package(PkgConfig REQUIRED)
find_library(mosquitto REQUIRED)
find_library(mosquittopp REQUIRED)

pkg_check_modules(MSGPACK REQUIRED msgpack)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_library(RPIMoCapBase STATIC)
target_sources(RPIMoCapBase
    PRIVATE
    src/Core/avahibrowser.cpp
    src/Core/frame.cpp
    src/Core/line3d.cpp
    src/Core/mqttsettings.cpp
    src/Core/mqttsubscriber.cpp
    PUBLIC
    include/RPIMoCap/Core/avahibrowser.h
    include/RPIMoCap/Core/cameraparams.h
    include/RPIMoCap/Core/frame.h
    include/RPIMoCap/Core/line3d.h
    include/RPIMoCap/Core/mqttpublisher.h
    include/RPIMoCap/Core/mqttsettings.h
    include/RPIMoCap/Core/mqttsubscriber.h
    include/RPIMoCap/Core/msgpack_defs.h)

target_include_directories(RPIMoCapBase PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(RPIMoCapBase PUBLIC
    mosquitto
    mosquittopp
    Qt5::Core
    Qt5::Network
    ${MSGPACK_LIBRARIES})
