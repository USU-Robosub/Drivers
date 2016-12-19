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
add_executable(run_basic test_basic_doppler.cpp)
add_executable(run_com test_doppler_commands.cpp)
add_executable(run_data test_doppler_data.cpp)

target_link_libraries(run_basic Drivers)
target_link_libraries(run_com Drivers)
target_link_libraries(run_data Drivers)