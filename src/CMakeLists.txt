add_library( xares SHARED xares.cpp )
target_link_libraries( xares ${LIBGLADYS_LIBRARIES} ${GDAL_LIBRARIES} )
install(TARGETS xares DESTINATION ${CMAKE_INSTALL_LIBDIR})
install_pkg_config_file(xares
    DESCRIPTION "eXploration module for Autonomous Robotic Embedded Systems"
    CFLAGS
    LIBS -lxares
    VERSION ${PACKAGE_VERSION})

#add_executable( foo foo.cpp )
#target_link_libraries( foo ${LIBGLADYS_LIBRARIES} ${GDAL_LIBRARIES} )
#install(TARGETS foo DESTINATION ${CMAKE_INSTALL_BINDIR})

