find_path(reconros_p1_INCLUDE_DIR reconros_p1.h HINTS /usr/local/include PATH_SUFFIXES reconros)
find_library(reconros_p1_LIBRARY NAMES reconros_p1 libreconros_p1 HINTS /usr/local/lib PATH_SUFFIXES reconros)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(reconros_p1 DEFAULT_MSG reconros_p1_LIBRARY reconros_p1_INCLUDE_DIR)
mark_as_advanced(reconros_p1_INCLUDE_DIR reconros_p1_LIBRARY)

set(reconros_p1_DEFINITIONS "")
set(reconros_p1_INCLUDE_DIRS ${reconros_p1_INCLUDE_DIR})
set(reconros_p1_LIBRARIES ${reconros_p1_LIBRARY})
