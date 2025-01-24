cmake_minimum_required(VERSION 2.8)
project(gici_ros)

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
  message_generation
)

add_message_files(
  FILES
  GlonassEphemeris.msg
  GnssAntennaPosition.msg
  GnssEphemerides.msg
  GnssEphemeris.msg
  GnssIonosphereParameter.msg
  GnssObservation.msg
  GnssObservations.msg
  GnssSsrCodeBias.msg
  GnssSsrCodeBiases.msg
  GnssSsrEphemerides.msg
  GnssSsrEphemeris.msg
  GnssSsrPhaseBias.msg
  GnssSsrPhaseBiases.msg
) 

generate_messages(
  DEPENDENCIES std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(../../.. gici.out)

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
aux_source_directory(src/ros_interface DIR_ROS_TOOLS)

# GICI ROS interface
add_executable(gici_ros_main
               src/gici_ros_main.cpp ${DIR_ROS_TOOLS})
target_link_libraries(gici_ros_main 
      gici ${OpenCV_LIBS} ${catkin_LIBRARIES})
