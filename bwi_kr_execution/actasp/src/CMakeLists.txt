
add_subdirectory(executors)
add_subdirectory(reasoners)
add_subdirectory(planners)

SET( actasp_SRC  ${actasp_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/Action.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/AspFluent.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/AnswerSet.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/MultiPolicy.cpp
PARENT_SCOPE)
