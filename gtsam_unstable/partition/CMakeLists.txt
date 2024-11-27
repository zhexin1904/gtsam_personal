# Install headers
file(GLOB partition_headers "*.h")
if(QNX)
    install(FILES ${partition_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/partition)
else()
    install(FILES ${partition_headers} DESTINATION include/gtsam_unstable/partition)
endif()

add_subdirectory(tests)
