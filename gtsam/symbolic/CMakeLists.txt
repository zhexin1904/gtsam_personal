# Install headers
file(GLOB symbolic_headers "*.h")
if(QNX)
    install(FILES ${symbolic_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/symbolic)
else()
    install(FILES ${symbolic_headers} DESTINATION include/gtsam/symbolic)
endif()

# Build tests
add_subdirectory(tests)
