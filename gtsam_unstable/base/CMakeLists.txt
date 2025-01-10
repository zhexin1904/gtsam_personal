# Install headers
file(GLOB base_headers "*.h")
if(QNX)
    install(FILES ${base_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/base)
else()
    install(FILES ${base_headers} DESTINATION include/gtsam_unstable/base)
endif()

# Add all tests
add_subdirectory(tests)
