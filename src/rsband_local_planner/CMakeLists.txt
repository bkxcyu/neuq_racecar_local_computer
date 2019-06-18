cmake_minimum_required(VERSION 2.8.3)
project(rsband_local_planner)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    angles
    base_local_planner
    cmake_modules
    costmap_2d
    dynamic_reconfigure
    eband_local_planner
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf_conversions
    sensor_msgs
    std_msgs
    visualization_msgs
    costmap_converter
)

find_package(OMPL REQUIRED)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(cfg/RSBandPlanner.cfg)

catkin_package(
  CATKIN_DEPENDS
    angles
    base_local_planner
    cmake_modules
    costmap_2d
    dynamic_reconfigure
    eband_local_planner
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf_conversions
    std_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

link_directories(/usr/local/lib ${catkin_INCLUDE_DIRS})

# build the library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}_ros.cpp
  src/fuzzy_ptc.cpp
  src/neuq_controller.cpp
  src/reeds_shepp_planner.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${OMPL_LIBRARIES}
  fuzzylite
  yaml-cpp
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  FILES blp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


add_library(neuq_controller
  include/rsband_local_planner/neuq_controller.h
  src/neuq_controller.cpp
)
add_dependencies(neuq_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(neuq_controller
  ${catkin_LIBRARIES}
)

## C++11 support
## Unfortunately, the 3d-party dependency libg2o requires c++11 starting from ROS Jade.
## Even if the ROS Jade specifications do not want c++11-only packages,
## we cannot compile without c++11 enabled. Another option would be to downgrade  
## libg2o third-party package.
## By now, if you do not want c++11, please refer to the ros indigo version.
IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support which is required 
  by linked third party packages starting from ROS Jade. Ignore this message for ROS Indigo.")
endif()
endif()


# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})
