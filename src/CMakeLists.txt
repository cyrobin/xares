add_library( xares SHARED xares.cpp )
target_link_libraries( xares ${LIBGLADYS_LIBRARIES} ${GDAL_LIBRARIES} )
install(TARGETS xares DESTINATION ${CMAKE_INSTALL_LIBDIR})
install_pkg_config_file(libxares
    DESCRIPTION "eXploration module for Autonomous Robotic Embedded Systems"
    CFLAGS
    LIBS -lxares
    VERSION ${PACKAGE_VERSION}
    REQUIRES "gladys >= ${GLADYS_MIN_VERSION}")

add_executable( xares_replay replay.cpp )
target_link_libraries( xares_replay xares ${LIBGLADYS_LIBRARIES} ${GDAL_LIBRARIES} )
install(TARGETS xares_replay DESTINATION ${CMAKE_INSTALL_BINDIR})

#add_executable( foo foo.cpp )
#target_link_libraries( foo xares ${LIBGLADYS_LIBRARIES} ${GDAL_LIBRARIES} )
#install(TARGETS foo DESTINATION ${CMAKE_INSTALL_BINDIR})

