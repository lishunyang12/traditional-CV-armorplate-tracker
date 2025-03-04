cmake_minimum_required(VERSION 3.16.3)

project(calibration)

set(CMAKE_CXX_STANDARD 17)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(main calibration.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${realsense2_LIBRARY})

