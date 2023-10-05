cmake_minimum_required(VERSION 2.8.3)
project(pips_visualization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pips_msgs
  pips
  pips_trajectory_testing
  roscpp
  std_msgs
  rviz
  rospy
  kdl_parser
  robot_state_publisher
  cv_bridge
  pcl_ros
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS pips_msgs #nav_msgs roscpp std_msgs
  DEPENDS rviz 
)






## This plugin includes Qt widgets, so we must include Qt like so:
#Qt4 on 14.04
#Qt5 on 16.04

message( "rviz version:" ${rviz_QT_VERSION})

##In theory, this should remove the need to use qt4/5_wrap_cpp
#set(CMAKE_AUTOMOC ON)

if (${rviz_QT_VERSION} VERSION_LESS "5.0")
  find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL)
  include(${QT_USE_FILE})
  
  ## Here we specify which header files need to be run through "moc",
  ## Qt's meta-object compiler.
  qt4_wrap_cpp(MOC_FILES1
    include/path_array_display.h
    include/robot_model_array_display.h
  )

  #qt4_wrap_cpp(MOC_FILES3
  #    include/pips_visualization/camera_pub_display.h
  #)

  
else()
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
  # set variable names already used with Qt4
  set(QT_LIBRARIES Qt5::Widgets)
  set(QTVERSION ${Qt5Widgets_VERSION})

    qt5_wrap_cpp(MOC_FILES1
    include/path_array_display.h
    include/robot_model_array_display.h
  )

endif()
add_definitions(-DQT_NO_KEYWORDS)


##Old method:
#find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
#include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
#add_definitions(-DQT_NO_KEYWORDS)





## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/path_array_display.cpp
  src/robot_model_array_display.cpp
  src/robot_hallucination.cpp
  #src/camera_pub_display.cpp
  ${MOC_FILES1}
  ${MOC_FILES3}
)




###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pips_rviz  ${SOURCE_FILES})

target_link_libraries(pips_rviz
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
  ${OGRE_OV_LIBRARIES_ABS}
 )


add_executable(test_display src/main.cpp)

target_link_libraries(test_display
  pips_rviz
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
  ${OGRE_OV_LIBRARIES_ABS}
  )


add_executable(point_cloud_to_mesh src/point_cloud_to_mesh.cpp)

target_compile_options(point_cloud_to_mesh PUBLIC  -std=c++14)

target_link_libraries(point_cloud_to_mesh
  ${catkin_LIBRARIES}
  )

  
add_executable(pose_marker_publisher src/pose_marker_publisher.cpp)
target_compile_options(pose_marker_publisher PUBLIC  -std=c++14)
target_link_libraries(pose_marker_publisher
  ${catkin_LIBRARIES}
  )

## Declare a C++ library
# add_library(pips_visualization
#   src/${PROJECT_NAME}/pips_visualization.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(pips_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(pips_visualization_node src/pips_visualization_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(pips_visualization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(pips_visualization_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pips_visualization pips_visualization_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pips_visualization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
