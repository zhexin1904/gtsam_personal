# Install headers
set (slam_excluded_headers #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/serialization.h"
)

file(GLOB slam_headers "*.h")
list(REMOVE_ITEM slam_headers ${slam_excluded_headers})
if(QNX)
    install(FILES ${slam_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/slam)
else()
    install(FILES ${slam_headers} DESTINATION include/gtsam_unstable/slam)
endif()

# Add all tests
add_subdirectory(tests)
