# Install headers
# Header groups
file(GLOB base_headers "*.h")
file(GLOB base_headers_tree "treeTraversal/*.h")

# Installation
if(NOT QNX)
    install(FILES ${base_headers} DESTINATION include/gtsam/base)
    install(FILES ${base_headers_tree} DESTINATION include/gtsam/base/treeTraversal)
else()
    # For QNX, Install in the Installation's Include Directory
    install(FILES ${base_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base)
    install(FILES ${base_headers_tree} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base/treeTraversal)
endif()

# Build tests
add_subdirectory(tests)
