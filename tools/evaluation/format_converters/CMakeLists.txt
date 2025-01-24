cmake_minimum_required(VERSION 2.8)
project(gici_formats)

# Set build flags. 
set(CMAKE_CXX_FLAGS "-std=c++11" )
set(CMAKE_C_FLAGS "-std=c99")

link_libraries(m dl rt)

# Add third-party libraries
# Eigen
find_package(Eigen3 REQUIRED NO_MODULE)
# Glog
find_package(glog REQUIRED)
# Opencv
find_package(OpenCV 4 REQUIRED)
# RTKLIB
add_definitions(-DENAGLO -DENACMP -DENAGAL -DENAQZS -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DDLL)
add_subdirectory(../../../third_party/rtklib rtklib.out)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  ../../../include
)

# Executables
add_executable(nmea_to_tum src/nmea_to_tum.cpp src/nmea_formator.cpp)
target_link_libraries(nmea_to_tum
                      rtklib
                      glog::glog)

add_executable(nmea_to_ie src/nmea_to_ie.cpp src/nmea_formator.cpp)
target_link_libraries(nmea_to_ie
                      rtklib
                      glog::glog)

add_executable(ie_to_nmea src/ie_to_nmea.cpp src/nmea_formator.cpp)
target_link_libraries(ie_to_nmea
                      rtklib
                      glog::glog)

add_executable(imupack_to_imr src/imupack_to_imr.cpp 
                      ../../../src/stream/format_imu.c)
target_link_libraries(imupack_to_imr
                      rtklib
                      glog::glog)

add_executable(imr_to_text src/imr_to_text.cpp)
target_link_libraries(imr_to_text
                      rtklib)

add_executable(imupack_to_text src/imupack_to_text.cpp 
                      ../../../src/stream/format_imu.c)
target_link_libraries(imupack_to_text
                      rtklib
                      glog::glog)

add_executable(imagepack_to_tum src/imagepack_to_tum.cpp 
                      ../../../src/stream/format_image.c)
target_link_libraries(imagepack_to_tum
                      rtklib
                      ${OpenCV_LIBS}
                      glog::glog)