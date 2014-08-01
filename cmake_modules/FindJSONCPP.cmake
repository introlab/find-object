# - Find JSONCPP
# This module finds an installed JSONCPP package.
#
# It sets the following variables:
#  JSONCPP_FOUND       - Set to false, or undefined, if JSONCPP isn't found.
#  JSONCPP_INCLUDE_DIRS - The JSONCPP include directory.
#  JSONCPP_LIBRARIES     - The JSONCPP library to link against.

FIND_PATH(JSONCPP_INCLUDE_DIRS json/features.h PATH_SUFFIXES jsoncpp)

FIND_LIBRARY(JSONCPP_LIBRARY NAMES jsoncpp)

IF (JSONCPP_INCLUDE_DIRS AND JSONCPP_LIBRARY)
   SET(JSONCPP_FOUND TRUE)
ENDIF (JSONCPP_INCLUDE_DIRS AND JSONCPP_LIBRARY)

IF (JSONCPP_FOUND)
   # show which JSONCPP was found only if not quiet
   SET(JSONCPP_LIBRARIES ${JSONCPP_LIBRARY})
   IF (NOT JSONCPP_FIND_QUIETLY)
      MESSAGE(STATUS "Found JSONCPP: ${JSONCPP_LIBRARIES}")
   ENDIF (NOT JSONCPP_FIND_QUIETLY)
ELSE (JSONCPP_FOUND)
   # fatal error if JSONCPP is required but not found
   IF (JSONCPP_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find JSONCPP (libjsoncpp)")
   ENDIF (JSONCPP_FIND_REQUIRED)
ENDIF (JSONCPP_FOUND)

