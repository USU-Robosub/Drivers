cmake_minimum_required(VERSION 2.8.9)

# Compiler Options Setup
project(Test)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror -Wextra -pedantic -lpthread")

# Includes
include_directories(../../Include/Interfaces)
include_directories(../../Include/Registers)
include_directories(../../Include/Sensors)
include_directories(../../Include/System)

# Created Executable Object
#add_library(drivers SHARED IMPORTED)

link_directories(../../Build)
add_executable(run BMP085_Test.cpp)
target_link_libraries(run Drivers)
