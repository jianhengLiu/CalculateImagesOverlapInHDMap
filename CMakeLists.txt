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

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

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


add_library(camera_model
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc
    )
target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


add_executable(${PROJECT_NAME} src/CalculateImagesOverlapInHDMap.cpp 
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
)

add_executable(CalImagesOverlapInHDMap_Euroc src/CalImagesOverlapInHDMap_Euroc.cpp 
)
target_link_libraries(CalImagesOverlapInHDMap_Euroc
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} camera_model
)

add_executable(Registration2DGridMapWith3DMap src/Registration2DGridMapWith3DMap.cpp 
)
target_link_libraries(Registration2DGridMapWith3DMap
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
)