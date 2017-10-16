cmake_minimum_required(VERSION 3.8)
project(Robot_Tour)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
set(SOURCE_FILES ../src/tourGuide.cpp )
LINK_DIRECTORIES(/usr/local/Aria/lib)
INCLUDE_DIRECTORIES(/usr/local/Aria/include ${OpenCV_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "-Wall")

add_executable(tourGuide ${SOURCE_FILES})
target_link_libraries(tourGuide libAria.so ${OpenCV_LIBS})




