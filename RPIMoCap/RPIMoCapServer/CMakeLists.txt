cmake_minimum_required(VERSION 3.1)

project(RPIMoCapServer CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -O3")

find_package(PkgConfig REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets 3DCore 3DRender 3DInput 3DLogic 3DExtras REQUIRED)
pkg_check_modules(MSGPACK REQUIRED msgpack)
pkg_check_modules(OPENCV REQUIRED opencv)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

QT5_WRAP_CPP(QOBJECT_HEADERS
    camerasettings.h
    camerasettingswidget.h)

set(SOURCES
    ${QOBJECT_HEADERS}
    camerasettingswidget.cpp
    linesaggregator.cpp
    mainwindow.cpp
    mocapscene3d.cpp
    munkres.cpp
    pointchecker.cpp
    rpimocapserver.cpp
    wandcalibration.cpp
    main.cpp)

add_executable(RPIMoCapServer ${SOURCES})

target_include_directories(RPIMoCapServer PUBLIC
    ${PROJECT_SOURCE_DIR})

target_link_libraries(RPIMoCapServer
    RPIMoCapBase
    ${MSGPACK_LIBRARIES}
    ${OPENCV_LIBRARIES}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::3DCore
    Qt5::3DRender
    Qt5::3DInput
    Qt5::3DLogic
    Qt5::3DExtras)
