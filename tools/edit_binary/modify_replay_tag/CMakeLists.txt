cmake_minimum_required(VERSION 2.8)
project(gici_tools)

# Set build flags. 
set(CMAKE_CXX_FLAGS "-std=c++11" )
set(CMAKE_CXX_FLAGS "-fpermissive")

# Add third-party libraries
# RTKLIB
add_definitions(-DENAGLO -DENAQZS -DENACMP -DENAGAL -DNFREQ=3 -DNEXOBS=3 -DDLL)
add_subdirectory(../../../third_party/rtklib rtklib.out)

include_directories(
  include
)

# Shift tag
add_executable(shift_tag src/shift_tag.cpp)
target_link_libraries(shift_tag 
                      rtklib)
