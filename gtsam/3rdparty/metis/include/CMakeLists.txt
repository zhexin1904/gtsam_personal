if(NOT QNX)
    install(FILES metis.h DESTINATION include/gtsam/3rdparty/metis)
else()
    #Install in the install include directory rather than at the install prefix.
    install(FILES metis.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/3rdparty/metis)
endif()