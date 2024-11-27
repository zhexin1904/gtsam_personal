# Install headers
file(GLOB nonlinear_headers "*.h")
if(QNX)
    install(FILES ${nonlinear_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/nonlinear)
else()
    install(FILES ${nonlinear_headers} DESTINATION include/gtsam_unstable/nonlinear)
endif()

# Add all tests
add_subdirectory(tests)
