cmake_minimum_required(VERSION 2.8.3)
project(dynamic_marker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs )

find_package(OpenCV REQUIRED)#2.4.13

message(STATUS "OpenCV library status:")
message(STATUS " version: ${OpenCV_VERSION}")
message(STATUS " libraries: ${OpenCV_LIBS}")
message(STATUS " include path: ${OpenCV_INCLUDE_DIRS}")
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
#find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES SchwarzWeissTest
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${image_geometry_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


## Declare a C++ executable
 add_executable(marker_main_node src/marker_main.cpp
    src/marker_generator.cpp
)

# Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(SchwarzWeissTest_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against


target_link_libraries(marker_main_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
#  ${image_geometry_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

