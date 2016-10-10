find_path(ALSA_INCLUDE_DIR NAMES asoundlib.h
          PATH_SUFFIXES alsa
          DOC "The ALSA (asound) include directory"
)

find_library(ALSA_LIBRARY NAMES asound
          DOC "The ALSA (asound) library"
)

# handle the QUIETLY and REQUIRED arguments and set ALSA_FOUND to TRUE if 
# all listed variables are TRUE
include("${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake")
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ALSA DEFAULT_MSG ALSA_LIBRARY ALSA_INCLUDE_DIR)

if(ALSA_FOUND)
  set( ALSA_LIBRARIES ${ALSA_LIBRARY} )
  set( ALSA_INCLUDE_DIRS ${ALSA_INCLUDE_DIR} )
endif()

mark_as_advanced(ALSA_INCLUDE_DIR ALSA_LIBRARY)
