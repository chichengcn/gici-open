cmake_minimum_required(VERSION 2.8)
project(gici)

# Set build flags. 
set(CMAKE_CXX_FLAGS "-std=c++11" )
set(CMAKE_CXX_FLAGS "-fpermissive")
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  if (IS_ARM)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
  else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
  endif ()
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
endif()

# Add third-party libraries
# Eigen
find_package(Eigen3 REQUIRED NO_MODULE)
# Opencv
find_package(OpenCV 4 REQUIRED)
# Yaml-cpp 
find_package(yaml-cpp REQUIRED)
# Glog
find_package(glog REQUIRED)
# Ceres-solver
find_package(Ceres REQUIRED)
# Fast feature detector
add_subdirectory(third_party/fast)
# SVO vikit tool-kits
add_subdirectory(third_party/rpg_vikit/vikit_common)
# SVO libraries
add_subdirectory(third_party/rpg_svo)
# RTKLIB
add_definitions(-DENAGLO -DENACMP -DENAGAL -DENAQZS -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DDLL)
add_subdirectory(third_party/rtklib)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

aux_source_directory(src/utility DIR_utility)
aux_source_directory(src/stream DIR_stream)
aux_source_directory(src/gnss DIR_gnss)
aux_source_directory(src/imu DIR_imu)
aux_source_directory(src/vision DIR_vision)
aux_source_directory(src/estimate DIR_estimate)
aux_source_directory(src/fusion DIR_fusion)
list(APPEND DIR_ALL ${DIR_utility}
                    ${DIR_stream}
                    ${DIR_gnss}
                    ${DIR_imu}
                    ${DIR_vision}
                    ${DIR_estimate}
                    ${DIR_fusion})

# library
add_library(${PROJECT_NAME} SHARED ${DIR_ALL})
target_link_libraries(${PROJECT_NAME} 
                      rtklib
                      vikit_common
                      svo
                      glog
                      ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

# executable
add_executable(gici_main src/gici_main.cpp)
target_link_libraries(gici_main 
                      ${PROJECT_NAME})
