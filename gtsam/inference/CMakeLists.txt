# Install headers
file(GLOB inference_headers "*.h")
if(QNX)
    install(FILES ${inference_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/inference)
else()
    install(FILES ${inference_headers} DESTINATION include/gtsam/inference)
endif()

# Build tests
add_subdirectory(tests)
