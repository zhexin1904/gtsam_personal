# Install headers
file(GLOB sam_headers "*.h")
if(QNX)
    install(FILES ${sam_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/sam)
else()
    install(FILES ${sam_headers} DESTINATION include/gtsam/sam)
endif()

# Build tests
add_subdirectory(tests)
