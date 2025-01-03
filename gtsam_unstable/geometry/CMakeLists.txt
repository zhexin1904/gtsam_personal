# Install headers
file(GLOB geometry_headers "*.h")
if(QNX)
    install(FILES ${geometry_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/geometry)
else()
    install(FILES ${geometry_headers} DESTINATION include/gtsam_unstable/geometry)
endif()

# Add all tests
add_subdirectory(tests)
