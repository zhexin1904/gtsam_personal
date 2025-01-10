# Install headers
set(subdir discrete)
file(GLOB discrete_headers "*.h")
# FIXME: exclude headers
if(QNX)
    install(FILES ${discrete_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/discrete)
else()
    install(FILES ${discrete_headers} DESTINATION include/gtsam/discrete)
endif()

# Add all tests
add_subdirectory(tests)
