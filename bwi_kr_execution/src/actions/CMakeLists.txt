

SET( kractions_SRC ${KREXEC_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/ActionFactory.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/LogicalNavigation.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/CallGUI.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/OpenDoor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/OpenSimulatedDoor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GoThrough.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ApproachDoor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/SearchRoom.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/AutomatedSearchRoom.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/AskPerson.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/AutomatedAskPerson.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Remind.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/AutomatedRemind.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/AutomatedPersonLocation.cpp
	PARENT_SCOPE)
