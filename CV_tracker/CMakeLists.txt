cmake_minimum_required(VERSION 3.16.3)

project(Lesson_5)

 

set(CMAKE_CXX_STANDARD 17)

 

# 查找 OpenCV 库

find_package(OpenCV REQUIRED)

# 查找 RealSense2 库

find_package(realsense2 REQUIRED)

# 查找 fmt 库

find_package(fmt REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})

# 添加可执行文件

add_executable(main src/main.cpp tasks/detector.cpp tasks/serial.cpp)

 

# 链接 OpenCV 库

target_link_libraries(main ${OpenCV_LIBS})

# 如果 realsense2 和 fmt 也需要链接到 main，则添加它们

target_link_libraries(main ${realsense2_LIBRARY})  # 注意：变量名可能因 find_package 的实现而异

target_link_libraries(main fmt::fmt)  # 对于 fmt 库，通常使用这种命名方式
