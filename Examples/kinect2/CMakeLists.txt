#BUILD kinect2
find_package(catkin REQUIRED COMPONENTS roscpp rostime std_msgs sensor_msgs message_filters cv_bridge image_transport compressed_image_transport compressed_depth_image_transport kinect2_bridge)
find_package(OpenMP)
find_package( Curses REQUIRED )
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${kinect2_bridge_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIRS}
)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(kinect2 kinect2.cpp)
target_link_libraries(kinect2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${kinect2_bridge_LIBRARIES}
  ${PROJECT_NAME}
  ${CURSES_LIBRARIES}
)

link_directories(/opt/ros/kinetic/lib)
