cmake_minimum_required(VERSION 2.8)
project(coordinate_converter)

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

# library
add_executable(ecef_to_lla src/ecef_to_lla.c)
target_link_libraries(ecef_to_lla rtklib)

add_executable(lla_to_ecef src/lla_to_ecef.c)
target_link_libraries(lla_to_ecef rtklib)

add_executable(deg_to_dms src/deg_to_dms.c)
target_link_libraries(deg_to_dms rtklib)

add_executable(lla_to_enu src/lla_to_enu.c)
target_link_libraries(lla_to_enu rtklib)

add_executable(ecef_to_enu src/ecef_to_enu.c)
target_link_libraries(ecef_to_enu rtklib)

add_executable(enu_to_lla src/enu_to_lla.c)
target_link_libraries(enu_to_lla rtklib)