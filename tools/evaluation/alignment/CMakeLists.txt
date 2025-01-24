cmake_minimum_required(VERSION 2.8)
project(alignment)

# Set build flags. 
set(CMAKE_CXX_FLAGS "-std=c++11" )
set(CMAKE_CXX_FLAGS "-fpermissive")
set(CMAKE_C_FLAGS "-std=c99")

add_subdirectory(../../.. gici.out)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  include
)

# Add definitions here to ensure correct memory allocation in RTKLIB
add_definitions(-DENAGLO -DENACMP -DENAGAL -DENAQZS -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DDLL)

# Executables
add_executable(nmea_align_timestamp src/nmea_align_timestamp.cpp
                                    src/nmea_formator.cpp)
target_link_libraries(nmea_align_timestamp
                      gici)

add_executable(nmea_estimate_pose_extrinsics src/nmea_estimate_pose_extrinsics.cpp
                                    src/nmea_formator.cpp)
target_link_libraries(nmea_estimate_pose_extrinsics
                      gici)

add_executable(nmea_pose_to_position src/nmea_pose_to_position.cpp 
                                     src/nmea_formator.cpp)
target_link_libraries(nmea_pose_to_position
                      gici)

add_executable(nmea_pose_to_pose src/nmea_pose_to_pose.cpp 
                      src/nmea_formator.cpp)
target_link_libraries(nmea_pose_to_pose
                      gici)