add_std_cxx11_flag("velodyne_tools.cc")
add_library(velodyne_tools
    velodyne_tools.cc)
target_link_libraries(velodyne_tools
    ${catkin_LIBRARIES})
#MESSAGE(STATUS *********** catkin_LIBRARIES: ${catkin_LIBRARIES})
install(TARGETS velodyne_tools
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
