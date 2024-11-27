# Install headers
file(GLOB navigation_headers "*.h")
if(QNX)
    install(FILES ${navigation_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/navigation)
else()
    install(FILES ${navigation_headers} DESTINATION include/gtsam/navigation)
endif()

# Add all tests
add_subdirectory(tests)
