cmake_minimum_required(VERSION 3.0.2)
project(dvs_displayer)

# ==== Build type ====
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)

IF(DEFINED CMAKE_BUILD_TYPE)
    SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
ENDIF()

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF()

IF(NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug" AND
    NOT ${CMAKE_BUILD_TYPE} STREQUAL "Release")
    MESSAGE(FATAL_ERROR "Only Release and Debug build types are allowed.")
ENDIF()

# ==== Depdendencies ====
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nodelet
    cv_bridge
    image_transport
    dvs_msgs
    dynamic_reconfigure

)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/dvs_displayer.cfg
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    include
    ${OpenCV_LIBRARY_DIRS}
)

file(GLOB_RECURSE SRC_INCS "include/${PROJECT_NAME}/*.h*")
file(GLOB_RECURSE SRC_SRCS "src/*.c*")
file(GLOB_RECURSE SRC_HDRS "src/*.h*")


add_library(${PROJECT_NAME} ${SRC_SRCS} ${SRC_HDRS} ${SRC_INCS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_node utils/displayer_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
