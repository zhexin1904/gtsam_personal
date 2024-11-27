# Install headers
file(GLOB dynamics_headers "*.h")
if(QNX)
    install(FILES ${dynamics_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/dynamics)
else()
    install(FILES ${dynamics_headers} DESTINATION include/gtsam_unstable/dynamics)
endif()

# Add all tests
add_subdirectory(tests)
