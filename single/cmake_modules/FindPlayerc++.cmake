# - Try to find playerc++
# Once done, this will define
#
#  Playerc++_FOUND - system has Playerc++
#  Playerc++_INCLUDE_DIRS - the Playerc++ include directories
#  Playerc++_LIBRARIES - link these to use Playerc++


find_package(PkgConfig)
pkg_check_modules(PC_PLAYERC++ QUIET playerc++)
set(PLAYERC++_DEFINITIONS ${PC_PLAYERC++_CFLAGS_OTHER})

find_path(PLAYERC++_INCLUDE_DIR libplayerc++/playerc++.h
          HINTS ${PC_PLAYERC++_INCLUDEDIR} ${PC_PLAYERC++_INCLUDE_DIRS}
          )
#          PATH_SUFFIXES libxml2 )

find_library(PLAYERC++_LIBRARY NAMES playerc++
             HINTS ${PC_PLAYERC++_LIBDIR} ${PC_PLAYERC++_LIBRARY_DIRS} )

set(PLAYERC++_LIBRARIES ${PLAYERC++_LIBRARY} )
set(PLAYERC++_INCLUDE_DIRS ${PLAYERC++_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Playerc++  DEFAULT_MSG
                                  PLAYERC++_LIBRARY PLAYERC++_INCLUDE_DIR)

mark_as_advanced(PLAYERC++_INCLUDE_DIR PLAYERC++_LIBRARY )
