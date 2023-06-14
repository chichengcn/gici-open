SET(PROJECT_NAME vikit_common)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
# SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(USE_ROS False) # Set False if you want to build this package without Catkin

# Add plain cmake packages 
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

IF(USE_ROS)
  find_package(catkin REQUIRED COMPONENTS roscpp cmake_modules)
  LIST(APPEND INCLUDE_DIRECTORIES ${catkin_INCLUDE_DIRS})
  catkin_package(
    DEPENDS Eigen OpenCV Sophus
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
ELSE()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# Set Sourcefiles
aux_source_directory(src/vikit_cameras CAMERAS_DIR)
aux_source_directory(src/vikit_common COMMON_DIR)
aux_source_directory(src/vikit_solver SOLVER_DIR)
set(SOURCEFILES ${CAMERAS_DIR} ${COMMON_DIR} ${SOLVER_DIR})

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${OpenCV_LIBS}
  yaml-cpp
  glog::glog)
# Link include directory
target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

IF(USE_ROS)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES})
ENDIF()

# Tests
# ADD_EXECUTABLE(test_vk_common test/test_common.cpp)
# TARGET_LINK_LIBRARIES(test_vk_common ${PROJECT_NAME})

# ADD_EXECUTABLE(test_vk_common_triangulation test/test_triangulation.cpp)
# TARGET_LINK_LIBRARIES(test_vk_common_triangulation ${PROJECT_NAME})

# ADD_EXECUTABLE(test_vk_common_patch_score test/test_patch_score.cpp)
# TARGET_LINK_LIBRARIES(test_vk_common_patch_score ${PROJECT_NAME})


################################################################################
# Create the vikit_commonConfig.cmake file for other cmake projects.
# IF(NOT USE_ROS)
#   GET_TARGET_PROPERTY( FULL_LIBRARY_NAME ${PROJECT_NAME} LOCATION )
#   SET(vikit_common_LIBRARIES ${FULL_LIBRARY_NAME} )
#   SET(vikit_common_LIBRARY_DIR ${PROJECT_BINARY_DIR} )
#   SET(vikit_common_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
#   CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
#       ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake @ONLY IMMEDIATE )
#   export( PACKAGE vikit_common )

#   INSTALL(DIRECTORY include/vikit DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
#   INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
# ENDIF()