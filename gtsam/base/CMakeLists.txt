# Install headers
file(GLOB base_headers "*.h")
if(QNX)
    install(FILES ${base_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base)
else()
    install(FILES ${base_headers} DESTINATION include/gtsam/base)
endif()

file(GLOB base_headers_tree "treeTraversal/*.h")
if(QNX)
    install(FILES ${base_headers_tree} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base/treeTraversal)
else()
    install(FILES ${base_headers_tree} DESTINATION include/gtsam/base/treeTraversal)
endif()

file(GLOB deprecated_headers "deprecated/*.h")
if(QNX)
    install(FILES ${deprecated_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base/deprecated)
else()
    install(FILES ${deprecated_headers} DESTINATION include/gtsam/base/deprecated)
endif()

# Build tests
add_subdirectory(tests)
