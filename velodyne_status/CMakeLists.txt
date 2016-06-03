#cmake_minimum_required(VERSION 2.8.3)
project(velodyne_status)

#find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    message_generation
    std_msgs)
    # dynamic_reconfigure)


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

# generate_dynamic_reconfigure_options(
#   cfg/VelodyneStatus.cfg
# )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

macro(add_flags SRC_FILES FLAGS)
#  MESSAGE("Flags list to add: ${FLAGS}")
#  MESSAGE("to this sources files:")
  foreach(src ${SRC_FILES})
#    MESSAGE("-> ${src}")
    set_source_files_properties(${src} PROPERTIES COMPILE_FLAGS ${FLAGS})
  endforeach()
endmacro()

macro(add_std_cxx14_flag SRC_FILES)
  add_flags("${SRC_FILES}" "-std=c++14")
endmacro()

macro(add_std_cxx11_flag SRC_FILES)
  add_flags("${SRC_FILES}" "-std=c++11")
endmacro()

include_directories(include ${catkin_INCLUDE_DIRS})
add_std_cxx11_flag("src/get_status_vlp_by_message.cpp")
add_executable(velodyne_status_server src/get_status_vlp_by_message.cpp)
target_link_libraries(velodyne_status_server ${catkin_LIBRARIES})
add_dependencies(velodyne_status_server ${catkin_EXPORTED_TARGETS})

# install runtime files
install(TARGETS velodyne_status_server
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
