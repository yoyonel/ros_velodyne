add_library(velodyne_settings vlp_webclient.cc)
target_link_libraries(velodyne_settings
                      ${catkin_LIBRARIES})
MESSAGE(STATUS *********** catkin_LIBRARIES: ${catkin_LIBRARIES})
install(TARGETS velodyne_settings
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
