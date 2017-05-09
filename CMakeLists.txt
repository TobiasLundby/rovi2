cmake_minimum_required(VERSION 2.8.3)
project(rovi2)
set(SHOULD_BUILD 1)
set(node_name "robot_node")

#include("cmake/use-c++11.cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rosconsole
  roscpp
  std_msgs
  cv_bridge
  image_transport
)


########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

########################################################################
#### RobWorkHardware
########################################################################
set(RWHW_ROOT "$ENV{RWHW_ROOT}")
find_package(RobWorkHardware COMPONENTS universalrobots PATHS "${RWHW_ROOT}")
if (NOT ROBWORKHARDWARE_FOUND)
  message(WARNING "Requires the universalrobots component of RobWorkHardware!")
  set(SHOULD_BUILD 0)
endif()


########################################################################
#### RobWorkStudio
########################################################################
SET(RWSTUDIO_ROOT $ENV{RWS_ROOT})
FIND_PACKAGE(RobWorkStudio REQUIRED PATHS "${RWSTUDIO_ROOT}")

########################################################################
#### Build process
########################################################################
if (SHOULD_BUILD)
  ################################################
  ## Declare ROS messages, services and actions ##
  ################################################
  add_service_files(
    FILES
    MovePtp.srv
    Plan.srv
    )

	add_message_files(
	  FILES
	  Q.msg
	  State.msg
    position2D.msg
    position3D.msg
	)

  generate_messages(
    DEPENDENCIES
    std_msgs
    )

  ###################################
  ## catkin specific configuration ##
  ###################################
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime roscpp std_msgs
    DEPENDS ROBWORK ROBWORKHARDWARE ROBWORKSTUDIO
    #CFG_EXTRAS "use-c++11.cmake"
    )

  ########################################################################
  #### Build
  ########################################################################
  include_directories(${node_name}
    include
    ${ROBWORK_INCLUDE_DIR}
    ${ROBWORKHARDWARE_INCLUDE_DIRS}
    ${ROBWORKSTUDIO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

  add_executable(${node_name}
    src/RobotNode.cpp
    src/node_main.cpp

    )

  target_link_libraries(${node_name}
    ${ROBWORK_LIBRARIES}
    ${ROBWORKHARDWARE_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARY_DIRS}
    ${catkin_LIBRARIES}
    )


  add_dependencies(${node_name} rovi2_generate_messages_cpp
    )

  #find_package( OpenCV REQUIRED ) # Required below
  add_executable(image_converter src/image_converter.cpp src/ColorDetector.cpp)
  #target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBS} )
  target_link_libraries(image_converter ${catkin_LIBRARIES} )
  add_dependencies(image_converter sensor_msgs cv_bridge roscpp std_msgs image_transport opencv rovi2_generate_messages_cpp)

  #find_package( OpenCV REQUIRED ) # Required below
  add_executable(ball_locator_3d src/ball_locator_3d.cpp src/ColorDetector.cpp)
  #target_link_libraries(ball_locator_3d ${catkin_LIBRARIES} ${OpenCV_LIBS} )
  target_link_libraries(ball_locator_3d ${catkin_LIBRARIES} )
  add_dependencies(ball_locator_3d sensor_msgs cv_bridge roscpp std_msgs image_transport opencv rovi2_generate_messages_cpp)

  #add_executable(camera3D src/camera3D.cpp src/Stereopsis.cpp)
  #target_link_libraries(camera3D ${catkin_LIBRARIES} ${OpenCV_LIBS))
  #add_dependencies(camera3D std_msgs rovi2_generate_messages_cpp)

  #add_executable(camera2D src/camera2D.cpp)
  #target_link_libraries(camera2D ${catkin_LIBRARIES})
  #add_dependencies(camera2D std_msgs)

  #add_executable(offboard_node src/offboard_node.cpp)
  #target_link_libraries(offboard_node ${catkin_LIBRARIES})
  #add_dependencies(offboard_node geometry_msgs mavros_msgs mavros roscpp rospy std_msgs)

  add_executable(Roadmap src/Roadmap.cpp src/Astar.cpp)
  target_link_libraries(Roadmap ${ROBWORK_LIBRARIES} ${catkin_LIBRARIES})
  add_dependencies(Roadmap std_msgs)


  #Robworkstudio plugin
  QT4_WRAP_UI(UIS_OUT_H src/StudioPlugin.ui)
  QT4_WRAP_CPP(MocSrcFiles include/StudioPlugin.hpp)
  QT4_ADD_RESOURCES(RccSrcFiles src/StudioPlugin_resources.qrc)
  set(StudioPlugin_src src/StudioPlugin.cpp)
  add_library(StudioPlugin MODULE ${StudioPlugin_src} ${UIS_OUT_H} ${MocSrcFiles} ${RccSrcFiles})
  target_link_libraries(StudioPlugin ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARY_DIRS} ${catkin_LIBRARIES})

catkin_package()

  ########################################################################
  #### Testing
  ########################################################################
else()
  message(WARNING "Skipping ${PROJECT_NAME} due to missing libraries - see earlier warnings.")
endif()
