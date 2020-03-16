cmake_minimum_required(VERSION 3.13)

project(RPIMoCapBase CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -O3")

find_package(Qt5 COMPONENTS Gui Widgets 3DCore 3DRender 3DInput 3DLogic 3DExtras REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(MSGPACK REQUIRED msgpack)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_library(RPIMoCapVisualizations STATIC)
target_sources(RPIMoCapVisualizations
    PRIVATE
    src/mocapscene3d.cpp
    src/mocapscene3d.ui
    src/primitives.cpp
    PUBLIC
    include/RPIMoCap/mocapscene3d.h
    include/RPIMoCap/primitives.h
)

target_include_directories(RPIMoCapVisualizations PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN_INCLUDE_DIR})

message(WARNING ${EIGEN3_INCLUDE_DIR})

target_link_libraries(RPIMoCapVisualizations PUBLIC
    RPIMoCapBase
    Qt5::Gui
    Qt5::Widgets
    Qt5::3DCore
    Qt5::3DRender
    Qt5::3DInput
    Qt5::3DLogic
    Qt5::3DExtras
    ${MSGPACK_LIBRARIES})
