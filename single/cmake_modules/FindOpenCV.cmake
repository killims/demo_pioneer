find_package(PkgConfig)
pkg_check_modules(PC_OPENCV QUIET opencv)
set(OPENCV_DEFINITIONS ${PC_OPENCV_CFLAGS_OTHER})

find_path(OPENCV_INCLUDE_DIR opencv/cv.h
          HINTS ${PC_OPENCV_INCLUDEDIR} ${PC_OPENCV_INCLUDE_DIRS}
          )
#          PATH_SUFFIXES libxml2 )

find_library(OPENCV_CORE_LIBRARY NAMES opencv_core
             HINTS ${PC_OPENCV_LIBDIR} ${PC_OPENCV_LIBRARY_DIRS} )

find_library(OPENCV_HIGHGUI_LIBRARY NAMES opencv_highgui
             HINTS ${PC_OPENCV_LIBDIR} ${PC_OPENCV_LIBRARY_DIRS} )

find_library(OPENCV_LEGACY_LIBRARY NAMES opencv_legacy
             HINTS ${PC_OPENCV_LIBDIR} ${PC_OPENCV_LIBRARY_DIRS} )

set(OPENCV_LIBRARIES ${OPENCV_CORE_LIBRARY} ${OPENCV_HIGHGUI_LIBRARY} ${OPENCV_LEGACY_LIBRARY})
set(OPENCV_INCLUDE_DIRS ${OPENCV_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(OpenCV DEFAULT_MSG
                                  OPENCV_LIBRARY OPENCV_INCLUDE_DIR)

mark_as_advanced(OPENCV_INCLUDE_DIR OPENCV_LIBRARY )
