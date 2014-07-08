project(actasp)

cmake_minimum_required(VERSION 2.8)

#change this to run the test 
set(UNITTESTPP_PATH "unittest++/UnitTest++.h")



include(CheckIncludeFileCXX)

include_directories(include)

 SET(actasp_SRC)

add_subdirectory(src)

add_library(actasp SHARED ${actasp_SRC})



# comple tests if UnitTest++ is available
UNSET(HAVE_UNITTESTXX CACHE)
CHECK_INCLUDE_FILE_CXX(${UNITTESTPP_PATH} HAVE_UNITTESTXX)

IF(HAVE_UNITTESTXX)

	add_executable(testAll test/src/main.cpp test/src/TestClingo.cpp test/src/TestAnswerSet.cpp test/src/TestAspFluent.cpp)
	target_link_libraries(testAll actasp libUnitTest++.a)

ENDIF()

