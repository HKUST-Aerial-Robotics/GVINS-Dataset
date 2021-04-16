cmake_minimum_required(VERSION 2.8.3)
project(gvins_dataset_toolkit)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    rosbag
    gnss_comm
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(bag2rinex src/bag2rinex.cpp)
target_link_libraries(bag2rinex ${catkin_LIBRARIES})

add_executable(bag2rtk_solution src/bag2rtk_solution.cpp)
target_link_libraries(bag2rtk_solution ${catkin_LIBRARIES})