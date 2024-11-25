cmake_minimum_required(VERSION 2.8.3)
project(ndtg_mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -Wno-error=deprecated-declarations -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  velodyne_pointcloud
  nmea_navsat_driver
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
# find_package(GTSAM REQUIRED QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL # GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs

  velodyne_pointcloud
  nmea_navsat_driver
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	# ${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  # ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# RTK Preintegration
add_executable(ndtg_mapping src/ndtg_mapping.cpp)
target_link_libraries(ndtg_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ) # gtsam)
add_dependencies(ndtg_mapping ${catkin_EXPORTED_TARGETS})

add_executable(queue_counter src/queue_counter.cpp)
target_link_libraries(queue_counter ${catkin_LIBRARIES})
add_dependencies(queue_counter ${catkin_EXPORTED_TARGETS})
