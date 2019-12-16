cmake_minimum_required(VERSION 3.13)

project(RPIMoCapServer CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -O3")

find_package(PkgConfig REQUIRED)
find_package(Qt5 COMPONENTS Gui Widgets 3DCore 3DRender 3DInput 3DLogic 3DExtras REQUIRED)
pkg_check_modules(MSGPACK REQUIRED msgpack)
pkg_check_modules(OPENCV REQUIRED opencv)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(RPIMoCapServer)
target_sources(RPIMoCapServer
    PRIVATE
    src/Server/camerasettingswidget.cpp
    src/Server/linesaggregator.cpp
    src/Server/mainwindow.cpp
    src/Server/mocapscene3d.cpp
    src/Server/munkres.cpp
    src/Server/pointchecker.cpp
    src/Server/rpimocapserver.cpp
    src/Server/wandcalibration.cpp
    src/Server/main.cpp
    PUBLIC
    include/RPIMoCap/Server/camerasettings.h
    include/RPIMoCap/Server/camerasettingswidget.h
    include/RPIMoCap/Server/linesaggregator.h
    include/RPIMoCap/Server/mainwindow.h
    include/RPIMoCap/Server/matrix.h
    include/RPIMoCap/Server/matrix.tpp
    include/RPIMoCap/Server/mocapscene3d.h
    include/RPIMoCap/Server/munkres.h
    include/RPIMoCap/Server/pointchecker.h
    include/RPIMoCap/Server/rpimocapserver.h
    include/RPIMoCap/Server/wandcalibration.h)

target_include_directories(RPIMoCapServer PUBLIC
    ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(RPIMoCapServer PRIVATE
    RPIMoCapBase
    ${MSGPACK_LIBRARIES}
    ${OPENCV_LIBRARIES}
    Qt5::Gui
    Qt5::Widgets
    Qt5::3DCore
    Qt5::3DRender
    Qt5::3DInput
    Qt5::3DLogic
    Qt5::3DExtras)
