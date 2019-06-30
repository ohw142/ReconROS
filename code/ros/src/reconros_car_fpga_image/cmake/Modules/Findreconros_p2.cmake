find_path(reconros_p2_INCLUDE_DIR reconros_p2.h HINTS /usr/local/include PATH_SUFFIXES reconros)
find_library(reconros_p2_LIBRARY NAMES reconros_p2 libreconros_p2 HINTS /usr/local/lib PATH_SUFFIXES reconros)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(reconros_p2 DEFAULT_MSG reconros_p2_LIBRARY reconros_p2_INCLUDE_DIR)
mark_as_advanced(reconros_p2_INCLUDE_DIR reconros_p2_LIBRARY)

set(reconros_p2_DEFINITIONS "")
set(reconros_p2_INCLUDE_DIRS ${reconros_p2_INCLUDE_DIR})
set(reconros_p2_LIBRARIES ${reconros_p2_LIBRARY})
