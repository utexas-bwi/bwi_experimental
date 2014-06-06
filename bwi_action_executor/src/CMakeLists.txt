
add_subdirectory(actions)

SET( ACTEXEC_SRC ${ACTEXEC_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/kr_interface.cpp
	PARENT_SCOPE)