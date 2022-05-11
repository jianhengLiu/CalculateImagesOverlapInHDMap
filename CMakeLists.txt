cmake_minimum_required(VERSION 3.0.2)
project(CalculateImagesOverlapInHDMap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ImagesRegistrationInHDMap
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/CalculateImagesOverlapInHDMap.cpp 
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
)
add_executable(Registration2DGridMapWith3DMap src/Registration2DGridMapWith3DMap.cpp 
)

## Specify libraries to link a library or executable target against
target_link_libraries(Registration2DGridMapWith3DMap
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
)

