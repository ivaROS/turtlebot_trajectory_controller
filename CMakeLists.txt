cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_trajectory_controller)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        nodelet
                                        pluginlib
                                        std_msgs
                                        kobuki_msgs
                                        yocs_controllers cmake_modules geometry_msgs)
find_package(Eigen3 REQUIRED)


catkin_package(INCLUDE_DIRS include
               LIBRARIES trajectory_controller_nodelet
               CATKIN_DEPENDS roscpp
                              nodelet
                              pluginlib
                              std_msgs
                              kobuki_msgs
                              yocs_controllers geometry_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(trajectory_controller_nodelet src/nodelet.cpp)
add_dependencies(trajectory_controller_nodelet kobuki_msgs_gencpp)
target_link_libraries(trajectory_controller_nodelet ${catkin_LIBRARIES})


##install(TARGETS bump_blink_controller_nodelet
##        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
##install(DIRECTORY include/${PROJECT_NAME}/
##        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
        
##install(DIRECTORY plugins
##        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
        
##install(DIRECTORY launch
##        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
