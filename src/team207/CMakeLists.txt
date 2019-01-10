cmake_minimum_required(VERSION 2.8.3)
project(team207)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
)

set(OpenCV_STATIC OFF)
find_package(  OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(	
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/detectlane.cpp
  src/carcontrol.cpp
  src/detect_traffic_sign.cpp
  src/polyfit.cpp
  src/cv_bridge/cv_bridge.cpp
  src/cv_bridge/rgb_colors.cpp
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  #${catkin_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)
