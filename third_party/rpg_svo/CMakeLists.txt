SET(PROJECT_NAME svo)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
# SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo

# Add plain cmake packages 
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Set Sourcefiles
aux_source_directory(src/svo_common COMMON_DIR)
aux_source_directory(src/svo_direct DIRECT_DIR)
aux_source_directory(src/rpg_common RPG_COMMON_DIR)
aux_source_directory(src/svo_tracker TRACKER_DIR)
# aux_source_directory(src/svo_img_align IMG_ALIGN_DIR)
list(APPEND IMG_ALIGN_DIR 
  src/svo_img_align/sparse_img_align_base.cpp
  src/svo_img_align/sparse_img_align.cpp)
aux_source_directory(src/svo_vio_common VIO_COMMON)
aux_source_directory(src/svo SVO_DIR)
set(SOURCEFILES ${COMMON_DIR} ${DIRECT_DIR} ${SVO_DIR}
  ${RPG_COMMON_DIR} ${TRACKER_DIR} ${IMG_ALIGN_DIR}
  ${VIO_COMMON})

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                      vikit_common
                      glog
                      fast
                      ${OpenCV_LIBS}
                      ${CERES_LIBRARIES})
# Link include directory
target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)


# Tests
# ADD_EXECUTABLE(test_depth_filter test/test_depth_filter.cpp)
# TARGET_LINK_LIBRARIES(test_depth_filter 
#                       ${PROJECT_NAME})

# ADD_EXECUTABLE(test_frame_handler test/test_frame_handler.cpp)
# TARGET_LINK_LIBRARIES(test_frame_handler 
#                       ${PROJECT_NAME})
