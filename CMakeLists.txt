cmake_minimum_required(VERSION 3.10)
project(barn_challenge_lu)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf2
    tf2_ros
    costmap_2d
    message_generation
    geometry_msgs
    std_msgs
)

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
find_package(Protobuf REQUIRED)

catkin_package(
    CATKIN_DEPENDS gazebo_msgs costmap_2d message_runtime tf2 tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(collision_check_publisher_node src/collision_publisher.cpp)

add_dependencies(collision_check_publisher_node
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

target_link_libraries(collision_check_publisher_node
   ${catkin_LIBRARIES}
 )

target_link_libraries(collision_check_publisher_node
                ${GAZEBO_LIBRARIES}
                ${Boost_LIBRARIES}
                ${PROTOBUF_LIBRARIES}
                pthread
)


#############################################################################
###################### SIMULATION REQUIREMENT ###############################
#############################################################################

#Find OpenGL, OpenCV and GLUT
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(GLUT REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

#############################################################################
#Find Other Libs

find_library(PNG_LIB png /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH})
find_library(DL_LIB dl /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH})

#############################################################################
include_directories(src)
aux_source_directory(src/Components           SRC_COMPONENTS)
aux_source_directory(src/External             SRC_EXTERNAL)
aux_source_directory(src/External/PQP         SRC_PQP)
aux_source_directory(src/Planners             SRC_PLANNERS)
aux_source_directory(src/Programs             SRC_PROGRAMS)
aux_source_directory(src/Robot                SRC_ROBOT)
aux_source_directory(src/SceneAndSimulation   SRC_SCENEANDSIMULATION)
aux_source_directory(src/Utils                SRC_UTILS)

add_executable(Runner
        ${SRC_UTILS}
        ${SRC_EXTERNAL}
        ${SRC_PQP}
        ${SRC_COMPONENTS}
        ${SRC_PLANNERS}
        ${SRC_PROGRAMS}
        ${SRC_ROBOT}
        ${SRC_SCENEANDSIMULATION}
)

add_dependencies(Runner
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

set_property(TARGET Runner PROPERTY ENABLE_EXPORTS 1)

target_link_libraries(Runner
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${DL_LIB}
        ${OPENGL_LIBRARIES}
        ${GLUT_glut_LIBRARY}
        ${PNG_LIB}
        m
        pthread
        stdc++fs)


MESSAGE("
*****************************************************************************
* EXTERNAL LIBRARIES
* GL      : ${OPENGL_LIBRARIES}
* GLUT   : ${GLUT_glut_LIBRARY}
* PNG_LIB: ${PNG_LIB}
* DL_LIB: ${DL_LIB}
*****************************************************************************
")

