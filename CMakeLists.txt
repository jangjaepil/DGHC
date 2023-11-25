cmake_minimum_required(VERSION 3.0.2)
project(dghc)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(qpOASES_DIR "$ENV{HOME}/qpOASES")
# set(DCHC_DIR "$ENV{HOME}/moca_ws/src/controllers/dghc") 
# set(WHQP_DIR "$ENV{HOME}/moca_ws/src/controllers/whqp") 

set(DCHC_DIR "$ENV{HOME}/master_harco_ws/src/controllers/dghc") 
set(WHQP_DIR "$ENV{HOME}/master_harco_ws/src/controllers/whqp") 

set(RA_INTERFACE_DIR "$ENV{HOME}/master_harco_ws/src/hrii_robot_interface/hrii_ra_interface") 
set(MOBILE_INTERFACE_DIR "$ENV{HOME}/master_harco_ws/src/hrii_robot_interface/hrii_mor_interface") 

find_package(pinocchio REQUIRED)
find_package(matlogger2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp 
  hrii_ra_interface
  hrii_mor_interface
  human_factor
  hrii_robot_msgs
  hrii_utils
)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${qpOASES_DIR}/include
  ${DCHC_DIR}/include/dghc
  ${WHQP_DIR}/include/whqp
  ${RA_INTERFACE_DIR}/include/hrii_ra_interface
  ${MOBILE_INTERFACE_DIR}/include/hrii_mor_interface
)

add_executable(dghc
  src/dghc.cpp
  src/GHCProjections.cpp
  src/dghc_controller.cpp
  
)

add_executable(dghc_fullbody
  src/fullbody/dghc_fullbody.cpp
  src/fullbody/GHCProjections_fullbody.cpp
  src/fullbody/dghc_controller_fullbody.cpp

)

add_executable(dghc_demo
  src/demo/dghc_demo.cpp
  src/demo/GHCProjections_demo.cpp
  src/demo/dghc_controller_demo.cpp

)

add_executable(dghc_alchemist
  src/alchemist/dghc_alchemist.cpp
  src/alchemist/GHCProjections_alchemist.cpp
  src/alchemist/dghc_controller_alchemist.cpp

)

add_executable(mode_pub_node
  src/mode_pub.cpp
)


add_executable(mode_pub_node_fullbody
  src/fullbody/mode_pub_fullbody.cpp
)

add_executable(mode_pub_node_demo
  src/demo/mode_pub_demo.cpp
)

add_executable(mode_pub_node_alchemist
  src/alchemist/mode_pub_alchemist.cpp
)

target_link_libraries(mode_pub_node 
  ${catkin_LIBRARIES}
)

target_link_libraries(mode_pub_node_fullbody 
  ${catkin_LIBRARIES}
)

target_link_libraries(mode_pub_node_demo 
  ${catkin_LIBRARIES}
)

target_link_libraries(mode_pub_node_alchemist 
  ${catkin_LIBRARIES}
)

target_link_libraries(dghc
  
  ${catkin_LIBRARIES}
  
  pinocchio::pinocchio
  matlogger2::matlogger2

)

target_link_libraries(dghc_fullbody
  
  ${catkin_LIBRARIES}
  
  pinocchio::pinocchio
  matlogger2::matlogger2

)

target_link_libraries(dghc_demo
  
  ${catkin_LIBRARIES}
  
  pinocchio::pinocchio
  matlogger2::matlogger2

)

target_link_libraries(dghc_alchemist
  
  ${catkin_LIBRARIES}
  
  pinocchio::pinocchio
  matlogger2::matlogger2

)




find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

