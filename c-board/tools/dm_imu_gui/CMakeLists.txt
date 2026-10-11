cmake_minimum_required(VERSION 3.16)

project(dm_imu_gui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets SerialPort)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets SerialPort)
find_package(Qt${QT_VERSION_MAJOR} QUIET COMPONENTS Charts)

add_executable(dm_imu_gui
    main.cpp
    mainwindow.h
    mainwindow.cpp
)

target_link_libraries(dm_imu_gui
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::SerialPort
)

if (TARGET Qt${QT_VERSION_MAJOR}::Charts)
    target_compile_definitions(dm_imu_gui PRIVATE HAVE_QT_CHARTS)
    target_link_libraries(dm_imu_gui Qt${QT_VERSION_MAJOR}::Charts)
endif()
