cmake_minimum_required(VERSION 3.13)

project(RPIMoCap CXX)

if (${CMAKE_CXX_COMPILER} MATCHES ".*gnueabihf.*")
    file(GLOB Qt_bin /usr/bin)
    find_program(QT_MOC_EXECUTABLE qt_moc moc PATHS ${Qt_bin})
    add_executable(Qt5::moc IMPORTED)
    set_property(TARGET Qt5::moc PROPERTY IMPORTED_LOCATION ${QT_MOC_EXECUTABLE})
endif ()

add_subdirectory(RPIMoCapBase)
add_subdirectory(RPIMoCapClient)

if (NOT ${CMAKE_CXX_COMPILER} MATCHES ".*gnueabihf.*")
    add_subdirectory(RPIMoCapServer)
    add_subdirectory(RPIMoCapVisualizations)
endif ()
