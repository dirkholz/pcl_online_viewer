cmake_minimum_required(VERSION 2.8.3)
project(pcl_online_viewer)
find_package(catkin REQUIRED)

catkin_package()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)

find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_libraries(${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pcl_online_viewer src/pcl_online_viewer.cpp)
