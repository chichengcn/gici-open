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

# Generate tag
add_executable(generate_tag src/generate_tag.cpp
                            src/utility.cpp)
target_link_libraries(generate_tag 
                      gici)
