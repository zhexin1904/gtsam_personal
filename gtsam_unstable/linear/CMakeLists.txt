# Install headers
file(GLOB linear_headers "*.h")
if(QNX)
    install(FILES ${linear_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/linear)
else()
    install(FILES ${linear_headers} DESTINATION include/gtsam_unstable/linear)
endif()

# Add all tests
add_subdirectory(tests)
