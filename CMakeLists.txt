cmake_minimum_required(VERSION 3.8)
project(Robot_Tour)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
set(SOURCE_FILES src/tourGuide.cpp src/eyesim.cpp include/eyesim.h src/screen.cpp include/screen.h )
LINK_DIRECTORIES(/usr/local/Aria/lib)
INCLUDE_DIRECTORIES(/usr/local/Aria/include ${OpenCV_INCLUDE_DIRS} include)
set(CMAKE_CXX_FLAGS "-Wall -g")

add_executable(tourGuide ${SOURCE_FILES})
target_link_libraries(tourGuide libAria.so ${OpenCV_LIBS})
