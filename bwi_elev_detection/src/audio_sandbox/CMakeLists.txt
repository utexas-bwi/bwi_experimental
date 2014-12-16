cmake_minimum_required(VERSION 2.8.7 FATAL_ERROR)
project(MY_GRAND_PROJECT)

include(src/audio_sandbox/FindPackageHandleStandardArgs.cmake)

include(src/audio_sandbox/FindALSA.cmake)

#OpenCV
find_package( OpenCV 2.0 REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})


#fftw - "libfftw3-3" and "libfftw3-dev"
find_library(FFTW_LIBRARY
     NAMES fftw3 fftw)
set(FFTW_LIBRARIES "${FFTW_LIBRARY}")
if(UNIX AND NOT WIN32)
     find_library(FFTW_libm_LIBRARY
         NAMES m)
     list(APPEND FFTW_LIBRARIES "${FFTW_libm_LIBRARY}")
endif()


#port audio - libportaudio19
include(src/audio_sandbox/FindPortaudio.cmake)
include_directories(${PORTAUDIO_INCLUDE_DIRS})
link_directories(${PORTAUDIO_LIBRARIES})
add_definitions(${PORTAUDIO_DEFINITIONS})

#alsa -- should already exist on new ubuntu machines
include(src/audio_sandbox/FindALSA.cmake)
include_directories(${ALSA_INCLUDE_DIR})
link_directories(${ALSA_LIBRARY})


# add_executable(mik_test src/mik_test.cpp)
# target_link_libraries(mik_test ${PORTAUDIO_LIBRARIES})


# add_executable(alsa_capture src/alsa_capture.cpp)
# target_link_libraries(alsa_capture ${ALSA_LIBRARY} ${FFTW_LIBRARIES}  ${OpenCV_LIBS} )

# add_executable(alsa_sound_detector_demo src/alsa_sound_detector_demo.cpp)
# target_link_libraries(alsa_sound_detector_demo ${ALSA_LIBRARY} ${FFTW_LIBRARIES}  ${OpenCV_LIBS} )
