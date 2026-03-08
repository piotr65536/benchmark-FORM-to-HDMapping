cmake_minimum_required(VERSION 3.10)
project(form_ros_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.8 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(TBB REQUIRED)

# FORM library — built at /opt/form-src/build during Docker image construction.
# Headers live directly under /opt/form-src so #include "form/form.hpp" resolves.
set(FORM_SRC_DIR /opt/form-src)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs std_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${FORM_SRC_DIR}
)

link_directories(${FORM_SRC_DIR}/build)

add_executable(form_ros_node src/form_ros_node.cpp)

add_dependencies(form_ros_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(form_ros_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  gtsam
  TBB::tbb
  FORM
)
