# Install headers
file(GLOB basis_headers "*.h")
if(QNX)
    install(FILES ${basis_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/basis)
else()
    install(FILES ${basis_headers} DESTINATION include/gtsam/basis)
endif()

# Build tests
add_subdirectory(tests)
