#cmake_minimum_required(VERSION 2.8.3)
project(velodyne_settings)

#find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    dynamic_reconfigure)


find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

generate_dynamic_reconfigure_options(
  cfg/VelodyneSettings.cfg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src/lib)

add_executable(velodyne_settings_server src/server.cpp)
target_link_libraries(velodyne_settings_server
    velodyne_settings
    ${catkin_LIBRARIES})
add_dependencies(velodyne_settings_server ${catkin_EXPORTED_TARGETS})

# install runtime files
install(TARGETS velodyne_settings_server
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     COMPONENT main
)
