cmake_minimum_required(VERSION 3.0.2)
project(school_classes)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_subscriber_node src/image_subscriber_node.cpp)

add_dependencies(image_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(image_subscriber_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS image_subscriber_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/image_subscriber_node.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


catkin_install_python(PROGRAMS
  scripts/capture_subscribed_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

