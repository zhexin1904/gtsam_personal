# Install headers
file(GLOB nonlinear_headers "*.h")
if(QNX)
    install(FILES ${nonlinear_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/nonlinear)
else()
    install(FILES ${nonlinear_headers} DESTINATION include/gtsam/nonlinear)
endif()

file(GLOB nonlinear_headers_internal "internal/*.h")
if(QNX)
    install(FILES ${nonlinear_headers_internal} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/nonlinear/internal)
else()
    install(FILES ${nonlinear_headers_internal} DESTINATION include/gtsam/nonlinear/internal)
endif()
# Build tests
add_subdirectory(tests)
