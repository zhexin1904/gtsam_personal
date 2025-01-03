# Install headers
set (slam_excluded_headers #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/serialization.h"
)

file(GLOB slam_headers "*.h")
if(QNX)
    install(FILES ${slam_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/slam)
else()
    install(FILES ${slam_headers} DESTINATION include/gtsam/slam)
endif()

# Build tests
add_subdirectory(tests)
