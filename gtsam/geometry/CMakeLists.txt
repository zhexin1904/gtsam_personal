# Install headers
file(GLOB geometry_headers "*.h")
if(QNX)
    install(FILES ${geometry_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/geometry)
else()
    install(FILES ${geometry_headers} DESTINATION include/gtsam/geometry)
endif()

# Build tests
add_subdirectory(tests)
