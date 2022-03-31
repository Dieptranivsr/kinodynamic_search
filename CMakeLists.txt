cmake_minimum_required(VERSION 2.8.3)
project(kinodynamic_search)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  plan_env
  path_searching
  #plan_manage

  traj_utils
  cv_bridge

  message_filters
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES kinodynamic_search #plan_manage
 CATKIN_DEPENDS plan_env #plan_manage #path_searching
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(kinodynamic_search
  src/main_node.cpp
  src/path_replan.cpp
  src/search_manager.cpp
  src/kinodynamic_search.cpp
  )

target_link_libraries(kinodynamic_search 
  ${catkin_LIBRARIES}
  #plan_manage
  )


#catkin_install_python(PROGRAMS scripts/getDepth.py scripts/depth_img.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)