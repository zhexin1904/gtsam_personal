# Install headers
file(GLOB discrete_headers "*.h")
if(QNX)
    install(FILES ${discrete_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/discrete)
else()
    install(FILES ${discrete_headers} DESTINATION include/gtsam_unstable/discrete)
endif()

# Add all tests
add_subdirectory(tests)

# Add examples
add_subdirectory(examples)
