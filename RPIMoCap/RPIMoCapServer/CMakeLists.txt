cmake_minimum_required(VERSION 3.13)

project(RPIMoCapServer CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -O3")

find_package(Ceres REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Qt5 COMPONENTS Gui Widgets 3DCore 3DRender 3DInput 3DLogic 3DExtras REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_library(RPIMoCapServerLib STATIC)
target_sources(RPIMoCapServerLib
    PRIVATE
    src/Server/camerasettings.cpp
    src/Server/camerasettingswidget.cpp
    src/Server/camerasettingswidget.ui
    src/Server/calibrationwidget.cpp
    src/Server/calibrationwidget.ui
    src/Server/linesaggregator.cpp
    src/Server/mainwindow.cpp
    src/Server/mainwindow.ui
    src/Server/pointchecker.cpp
    src/Server/rpimocapserver.cpp
    src/Server/wanddetector.cpp
    src/Server/wandcalibration.cpp
    src/Server/floorcalibrationwidget.cpp
    src/Server/floorcalibrationwidget.ui
    PUBLIC
    src/Server/camerasettings.h
    src/Server/camerasettingswidget.h
    src/Server/calibrationwidget.h
    src/Server/linesaggregator.h
    src/Server/mainwindow.h
    src/Server/pointchecker.h
    src/Server/rpimocapserver.h
    src/Server/wanddetector.h
    src/Server/wandcalibration.h
    src/Server/floorcalibrationwidget.h)

target_include_directories(RPIMoCapServerLib PUBLIC
    ${PROJECT_SOURCE_DIR}/src)

target_link_libraries(RPIMoCapServerLib PUBLIC
    RPIMoCapBase
    RPIMoCapVisualizations
    ${OPENCV_LIBRARIES}
    ${CERES_LIBRARIES}
    Qt5::Gui
    Qt5::Widgets
    Qt5::3DCore
    Qt5::3DRender
    Qt5::3DInput
    Qt5::3DLogic
    Qt5::3DExtras)

add_executable(RPIMoCapServer src/Server/main.cpp)
target_link_libraries(RPIMoCapServer RPIMoCapServerLib)

add_subdirectory(test)
