cmake_minimum_required(VERSION 2.8)
project(gici_tools)

# Set build flags. 
set(CMAKE_CXX_FLAGS "-std=c++11" )
set(CMAKE_CXX_FLAGS "-fpermissive")

# GICI
add_subdirectory(../../.. gici.out)

include_directories(
  include
)

# Add definitions here to ensure correct memory allocation in RTKLIB
add_definitions(-DENAGLO -DENACMP -DENAGAL -DENAQZS -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DDLL)

# Correct IMU timestamp
add_executable(correct_imu_timestamp src/correct_imu_timestamp.cpp
                                     src/edit_timestamp_utility.cpp)
target_link_libraries(correct_imu_timestamp 
                      gici)

# Correct IMU timestamp
add_executable(create_iono_parameter src/create_iono_parameter.cpp
                                     src/edit_timestamp_utility.cpp)
target_link_libraries(create_iono_parameter 
                      gici)

# Cut files
add_executable(cut_files src/cut_files.cpp
                         src/edit_timestamp_utility.cpp)
target_link_libraries(cut_files 
                      gici)

# Cut files
add_executable(check_image_timestamp src/check_image_timestamp.cpp)
target_link_libraries(check_image_timestamp 
                      gici)