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

find_package(Qt5 COMPONENTS Widgets Core Network SerialPort REQUIRED)
#find_package(VTK REQUIRED)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)
set(${OpenCV_DIR} /home/tapati/OpenCV/build-opencv-Desktop-Default)
#find_package(OpenCV 4.5.1 COMPONENTS core surface_matching)
find_package(OpenCV 4.5.1 REQUIRED)
find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)
# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${SOURCE_DIR} ${PCL_INCLUDE_DIRS} ${realsense2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS}  ${VISP_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(6DPoseEstimation
    mainwindow.ui
    mainwindow.cpp
    main.cpp
    camera.cpp
    pclviewer.cpp
    handeyecalibration.cpp
    motoudp.cpp
    convert.cpp
    servocontrol.cpp
    ppf.cpp
    yolo.cpp
)
#add_library(lib6DPoseEstimation SHARED
#    mainwindow.ui
#    mainwindow.cpp
#    camera.cpp
#    pclviewer.cpp
#    handeyecalibration.cpp
#    motoudp.cpp
#    convert.cpp
#    servocontrol.cpp
#    ppf.cpp
#    yolo.cpp
#    )
# Make sure the compiler can find include files for our lib6DPoseEstimation library
# when other libraries or executables link to Hello
#target_include_directories (lib6DPoseEstimation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
#qt5_use_modules(6DPoseEstimation Core SerialPort)
#target_link_libraries(lib6DPoseEstimation ${realsense2_LIBRARY} Qt5::Widgets Qt5::Network Qt5::SerialPort ${PCL_LIBRARIES} ${OpenCV_LIBS} ${VISP_LIBRARIES})
target_link_libraries(6DPoseEstimation ${realsense2_LIBRARY} Qt5::Widgets Qt5::Network Qt5::SerialPort ${PCL_LIBRARIES} ${OpenCV_LIBS} ${VISP_LIBRARIES})
