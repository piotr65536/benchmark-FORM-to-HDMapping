cmake_minimum_required(VERSION 3.0.2)
project(form-to-hdmapping)

find_package(catkin QUIET COMPONENTS roscpp)

if(catkin_FOUND)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  rosbag
  pcl_ros
)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs rosbag pcl_ros
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_subdirectory(src/3rdparty/LASzip)
include_directories(src/3rdparty/LASzip/include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(listener
  src/form_odometry_subscriber.cpp
  src/laz_writer.cpp
)

add_dependencies(listener
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(listener
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  laszip
  ${PCL_LIBRARIES}
)

else()
  message("catkin not found so skipping this package")
endif()
