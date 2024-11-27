# Install headers
file(GLOB linear_headers "*.h")
if(QNX)
    install(FILES ${linear_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/linear)
else()
    install(FILES ${linear_headers} DESTINATION include/gtsam/linear)
endif()

# Build tests
add_subdirectory(tests)
