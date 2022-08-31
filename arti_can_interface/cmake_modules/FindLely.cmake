# - Try to find LELY
# Once done this will define
#  LELY_FOUND - System has LELY
#  LELY_INCLUDE_DIR - The LELY include directories
#  LELY_LIBRARIES_DIR - The libraries needed to use LELY
#  LELY_DEFINITIONS - Compiler switches required for using LELY

find_package(PkgConfig)
pkg_check_modules(PC_LELY liblely-coapp)
set(LELY_DEFINITIONS ${PC_LELY_CFLAGS_OTHER})

find_path(LELY_INCLUDE_DIR lely/features.h
  HINTS ${PC_LELY_INCLUDEDIR} ${PC_LELY_INCLUDE_DIRS}
  PATH_SUFFIXES lely )

FOREACH(LIB ${PC_LELY_LIBRARIES})
  FIND_LIBRARY(FOUND_LIB_${LIB} ${LIB})
  LIST(APPEND LELY_LIBRARIES_DIR ${FOUND_LIB_${LIB}})
ENDFOREACH(LIB)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LELY_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LELY  DEFAULT_MSG
  LELY_LIBRARIES_DIR LELY_INCLUDE_DIR)

