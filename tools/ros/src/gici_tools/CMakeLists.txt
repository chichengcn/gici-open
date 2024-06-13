cmake_minimum_required(VERSION 2.8)
project(gici_tools)

set(CMAKE_CXX_FLAGS "-std=c++11" )
set(CMAKE_CXX_FLAGS "-fpermissive")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -O3 -ansi -pedantic")
set(CMAKE_C_FLAGS "-Wno-unused-but-set-variable")

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  tf
  tf_conversions
  cv_bridge
  image_transport
  eigen_conversions
  pcl_conversions
  pcl_ros
  rosbag
  message_generation
  gici_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(../../../.. gici.out)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS roscpp std_msgs nav_msgs
        sensor_msgs geometry_msgs 
        tf tf_conversions cv_bridge
        image_transport eigen_conversions
        message_generation
  DEPENDS EIGEN3 OpenCV
)

# Add definitions here to ensure correct memory allocation in RTKLIB
add_definitions(-DENAGLO -DENACMP -DENAGAL -DENAQZS -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DDLL)

# Sources
aux_source_directory(src/ros_utility DIR_UTILITIES)

# Executables
add_executable(publish_ie_poses
              src/publish_ie_poses.cpp ${DIR_UTILITIES})
target_link_libraries(publish_ie_poses 
              gici ${catkin_LIBRARIES})

add_executable(publish_tum_poses
               src/publish_tum_poses.cpp ${DIR_UTILITIES})
target_link_libraries(publish_tum_poses 
              gici ${catkin_LIBRARIES})

add_executable(publish_nmea_poses
              src/publish_nmea_poses.cpp ${DIR_UTILITIES})
target_link_libraries(publish_nmea_poses 
              gici ${catkin_LIBRARIES})

add_executable(imupack_to_rosbag
              src/imupack_to_rosbag.cpp)
target_link_libraries(imupack_to_rosbag 
              gici ${catkin_LIBRARIES})
              
add_executable(imagepack_to_rosbag
              src/imagepack_to_rosbag.cpp)
target_link_libraries(imagepack_to_rosbag 
              gici ${catkin_LIBRARIES})

add_executable(nmea_to_rosbag_navsatfix
              src/nmea_to_rosbag_navsatfix.cpp ${DIR_UTILITIES})
target_link_libraries(nmea_to_rosbag_navsatfix 
              gici ${catkin_LIBRARIES})

add_executable(gici_files_to_rosbag
              src/gici_files_to_rosbag.cpp ${DIR_UTILITIES})
target_link_libraries(gici_files_to_rosbag 
              gici ${catkin_LIBRARIES})