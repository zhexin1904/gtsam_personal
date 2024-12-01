
###############################################################################
# Find Google perftools
find_package(GooglePerfTools)
target_link_libraries(${PROJECT_NAME} ${GPERFTOOLS_TCMALLOC} ${GPERFTOOLS_PROFILER})