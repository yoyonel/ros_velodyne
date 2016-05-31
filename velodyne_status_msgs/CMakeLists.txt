#cmake_minimum_required(VERSION 2.8.3)
project(velodyne_status_msgs)

#find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    message_generation
    std_msgs)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

add_message_files(
  DIRECTORY msg
  FILES
  VelodyneStatus.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ServiceVelodyneStatus.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(velodyne_status_server src/get_status_vlp_by_message.cpp)
target_link_libraries(velodyne_status_server ${catkin_LIBRARIES})
add_dependencies(velodyne_status_server ${catkin_EXPORTED_TARGETS})

# install runtime files
install(TARGETS velodyne_status_server
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
