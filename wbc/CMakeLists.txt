cmake_minimum_required(VERSION 3.0.2)
project(wbc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(catkin REQUIRED)
find_package(Eigen3)
find_package(roscpp)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS system thread)

set(LIB_NAME ${PROJECT_NAME})

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               DEPENDS EIGEN3)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIR}
 ${Eigen3_INCLUDE_DIR}
)
include_directories ("${PROJECT_SOURCE_DIR}/")
include_directories ("${PROJECT_SOURCE_DIR}/lib/")

file (GLOB SRC_URDF lib/urdf/*.cpp)
add_library(URDF ${SRC_URDF})

file (GLOB SRC_RBDL lib/rbdl/*.cc)
add_library(RBDL ${SRC_RBDL})
target_link_libraries(RBDL URDF)

file(GLOB SRC_UTILS utils/*.cpp)
add_library(Utils ${SRC_UTILS})

file (GLOB SRC_ROBOT_MODEL model/*.cpp)
add_library(RobotModel ${SRC_ROBOT_MODEL})
target_link_libraries(RobotModel RBDL Utils)

add_executable(wbc_test src/MAIN.cpp)
target_link_libraries(wbc_test RobotModel)

add_library(${PROJECT_NAME}
    model/DynModel.cpp
    model/KinModel.cpp
    model/RobotModel.cpp
    utils/common_convert_calcu.cpp
    utils/pseudo_inverse.cpp
    ${SRC_ROBOT_MODEL})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
)
