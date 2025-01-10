# Install headers
file(GLOB sfm_headers "*.h")
if(QNX)
    install(FILES ${sfm_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/sfm)
else()
    install(FILES ${sfm_headers} DESTINATION include/gtsam/sfm)
endif()

# Build tests
add_subdirectory(tests)
