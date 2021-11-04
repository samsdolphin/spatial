cmake_minimum_required(VERSION 3.1)
project(spatial)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  pcl_conversions
  pcl_ros
  livox_ros_driver
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs image_transport sensor_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS
)

add_executable(extractor src/extractor.cpp)
target_link_libraries(extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(reconstruct src/reconstruct.cpp)
target_link_libraries(reconstruct ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(parse_livox_time src/parse_livox.cpp)
target_link_libraries(parse_livox_time ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_INCLUDE_DIRS})

add_executable(parse_camera_time src/parse_camera.cpp)
target_link_libraries(parse_camera_time ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_INCLUDE_DIRS})

add_executable(align_cam_time src/align_cam_time.cpp)
target_link_libraries(align_cam_time ${catkin_LIBRARIES} ${PCL_LIBRARIES})
