cmake_minimum_required(VERSION 3.1.0)

project(6DPoseEstimation VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Widgets Core Network REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)
# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${SOURCE_DIR} ${PCL_INCLUDE_DIRS} ${realsense2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(6DPoseEstimation
    mainwindow.ui
    mainwindow.cpp
    main.cpp
    camera.cpp
    pclviewer.cpp
    handeyecalibration.cpp
    motoudp.cpp
    convert.h
    #resources.qrc
)

target_link_libraries(6DPoseEstimation ${realsense2_LIBRARY} Qt5::Widgets Qt5::Network ${PCL_LIBRARIES} ${OpenCV_LIBS} ${VISP_LIBRARIES})
