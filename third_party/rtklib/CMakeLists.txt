cmake_minimum_required(VERSION 2.8)
project(rtklib)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -O3 -ansi -pedantic")
set(CMAKE_C_FLAGS "-Wno-unused-but-set-variable -Wno-format-overflow -Wno-unused-result")

include_directories(
    include
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

aux_source_directory(src DIR_SRCS_RTKLIB)
aux_source_directory(src/rcv DIR_SRCS_RTKLIB_RCV)
list(APPEND DIR_SRCS ${DIR_SRCS_RTKLIB} ${DIR_SRCS_RTKLIB_RCV})

add_library(${PROJECT_NAME} SHARED ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME} pthread)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)
