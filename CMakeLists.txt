cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_trajectory_controller)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        nodelet
                                        pluginlib
                                        std_msgs
                                        kobuki_msgs
                                        yocs_controllers cmake_modules geometry_msgs 
                              trajectory_generator
                                        )
##Note: on indigo, must be 'Eigen3' here and under 'include_directories'. Hydro, just 'Eigen'
find_package(Eigen REQUIRED)

catkin_package(INCLUDE_DIRS include
               LIBRARIES trajectory_controller_nodelet
               CATKIN_DEPENDS roscpp
                              nodelet
                              pluginlib
                              std_msgs
                              kobuki_msgs
                              yocs_controllers geometry_msgs 

                              trajectory_generator
                              )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -std=c++11")


add_library(trajectory_controller_nodelet src/nodelet.cpp)
add_dependencies(trajectory_controller_nodelet kobuki_msgs_gencpp)
target_link_libraries(trajectory_controller_nodelet ${catkin_LIBRARIES})

#add_library(Trajectory_Tester src/send_test_trajectory.cpp)
#target_link_libraries(Trajectory_Tester ${catkin_LIBRARIES})

##install(TARGETS bump_blink_controller_nodelet
##        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
##install(DIRECTORY include/${PROJECT_NAME}/
##        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
        
##install(DIRECTORY plugins
##        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
        
##install(DIRECTORY launch
##        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
