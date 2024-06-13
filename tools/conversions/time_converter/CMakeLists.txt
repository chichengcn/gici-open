cmake_minimum_required(VERSION 2.8)
project(time_converter)

# Set build flags. 
set(CMAKE_C_FLAGS "-std=c99")

link_libraries(m dl rt)

# Add third-party libraries
# RTKLIB
add_definitions(-DENAGLO -DENACMP -DENAGAL -DENAQZS -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DDLL)
add_subdirectory(../../../third_party/rtklib rtklib.out)

include_directories(
  include
)

# executable
add_executable(gpst_to_unix src/gpst_to_unix.c)
target_link_libraries(gpst_to_unix rtklib)

add_executable(unix_to_gpst src/unix_to_gpst.c)
target_link_libraries(unix_to_gpst rtklib)

add_executable(unix_to_epoch src/unix_to_epoch.c)
target_link_libraries(unix_to_epoch rtklib)