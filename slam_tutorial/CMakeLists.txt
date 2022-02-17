cmake_minimum_required(VERSION 3.0.2)
project(slam_tutorial)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shimizu_abstraction
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_executable(ceres_spg src/ceres_spg.cpp)
target_link_libraries(ceres_spg ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(gtsam_spg src/gtsam_spg.cpp)
target_link_libraries(gtsam_spg ${catkin_LIBRARIES} ${GTSAM_LIBRARIES} ${Boost_LIBRARIES})