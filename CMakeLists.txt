cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_colour_registration)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES colour_registration
  CATKIN_DEPENDS cv_bridge nodelet pcl_conversions pcl_ros roscpp sensor_msgs tf tf_conversions
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(pointcloud_colour_registration_node
  src/colour_registration_node.cpp

  src/colour_registration_st.cpp
  include/colour_registration/colour_registration_st.h
)
target_link_libraries(pointcloud_colour_registration_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(pointcloud_colour_registration_node ${catkin_EXPORTED_TARGETS})
