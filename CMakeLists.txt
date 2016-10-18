cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_trajectory_controller)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        nodelet
                                        pluginlib
                                        std_msgs
                                        kobuki_msgs
                                        yocs_controllers 
                                        cmake_modules 
                                        geometry_msgs 
                                        trajectory_generator
                                        tf2_ros
                                        tf2
                                        dynamic_reconfigure
                                        )

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/TurtlebotController.cfg
  )

catkin_package(INCLUDE_DIRS include
               LIBRARIES trajectory_controller
               CATKIN_DEPENDS roscpp
                              nodelet
                              pluginlib
                              std_msgs
                              kobuki_msgs
                              yocs_controllers 
                              geometry_msgs 
                              trajectory_generator
                              tf2_ros
                              tf2
               DEPENDS   
                              Eigen
                              )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

add_library(trajectory_controller 
   src/trajectory_controller.cpp)
add_dependencies(trajectory_controller 
   kobuki_msgs_gencpp
   ${PROJECT_NAME}_gencfg
   )
   
target_link_libraries(trajectory_controller 
   ${catkin_LIBRARIES})

add_executable(trajectory_controller_node
   src/turtlebot_controller_node.cpp )
add_dependencies(trajectory_controller_node
   trajectory_controller)
target_link_libraries(trajectory_controller_node
   trajectory_controller ${catkin_LIBRARIES})

add_library(trajectory_controller_nodelet MODULE
   src/nodelet.cpp )
add_dependencies(trajectory_controller_nodelet
   trajectory_controller)
target_link_libraries(trajectory_controller_nodelet
   trajectory_controller ${catkin_LIBRARIES})

add_executable(trajectory_sender
   src/send_test_trajectory.cpp)
target_link_libraries(trajectory_sender
   ${catkin_LIBRARIES})

##install(TARGETS bump_blink_controller_nodelet
##        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
##install(DIRECTORY include/${PROJECT_NAME}/
##        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
        
##install(DIRECTORY plugins
##        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
        
##install(DIRECTORY launch
##        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
