

SET( krexec_SRC ${krexec_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/action_execution.cpp
	PARENT_SCOPE)
	
SET( krreasoner_SRC ${krreasoner_SRC}
  ${CMAKE_CURRENT_SOURCE_DIR}/knowledge_reasoning.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/msgs_utils.cpp
PARENT_SCOPE)