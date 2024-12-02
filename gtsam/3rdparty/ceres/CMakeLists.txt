file(GLOB ceres_headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
if(NOT QNX)
    install(FILES ${ceres_headers} DESTINATION include/gtsam/3rdparty/ceres)
else()
    #Install in the install include directory rather than at the install prefix.
    install(FILES ${ceres_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/3rdparty/ceres)
endif()