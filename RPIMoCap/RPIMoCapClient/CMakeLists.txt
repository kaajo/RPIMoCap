cmake_minimum_required(VERSION 3.13)

project(RPIMoCapClient CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -O3")

find_package(PkgConfig REQUIRED)
find_package(Qt5 COMPONENTS Gui Widgets Concurrent REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_library(RPIMoCapClientLib STATIC ${QOBJECT_HEADERS} ${SOURCES})
target_sources(RPIMoCapClientLib
    PRIVATE
    src/ClientLib/markerdetector.cpp
    src/ClientLib/rpicamera.cpp
    src/ClientLib/client.cpp
    PUBLIC
    include/RPIMoCap/ClientLib/markerdetector.h
    include/RPIMoCap/ClientLib/rpicamera.h
    include/RPIMoCap/ClientLib/client.h
    INTERFACE
    include/RPIMoCap/ClientLib/icamera.h)

target_include_directories(RPIMoCapClientLib PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${GSTREAMER_BASE_INCLUDE_DIRS})

target_link_libraries(RPIMoCapClientLib PUBLIC
    RPIMoCapBase
    ${OPENCV_LIBRARIES}
    ${GSTREAMER_BASE_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    Qt5::Gui
    Qt5::Concurrent)

add_executable(RPIMoCapClient)
target_sources(RPIMoCapClient PRIVATE src/Client/main.cpp)
target_link_libraries(RPIMoCapClient RPIMoCapClientLib)

add_executable(RPIMoCapSimClient)
target_sources(RPIMoCapSimClient
    PRIVATE
    src/SimClient/main.cpp
    src/SimClient/simcamera.cpp
    src/SimClient/simcamerawidget.cpp
    src/SimClient/simscene.cpp
    src/SimClient/virtualwand.cpp
    src/SimClient/mainwindow.cpp
    src/SimClient/mainwindow.ui
    src/SimClient/simcamerawidget.ui
    PUBLIC
    include/RPIMoCap/SimClient/simmarker.h
    include/RPIMoCap/SimClient/simcamera.h
    include/RPIMoCap/SimClient/simscene.h
    include/RPIMoCap/SimClient/simcamerawidget.h
    include/RPIMoCap/SimClient/mainwindow.h
    include/RPIMoCap/SimClient/virtualwand.h)

target_link_libraries(RPIMoCapSimClient PRIVATE
    RPIMoCapClientLib
    Qt5::Widgets)

add_subdirectory(test)
