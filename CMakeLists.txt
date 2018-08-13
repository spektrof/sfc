#On Windows:
#cmake -G "Visual Studio 15 2017 Win64" -H. -Bbuild  
#cmake --build build --config Release 

cmake_minimum_required(VERSION 3.1)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(sfc)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

#----------------------------------
# Boost
find_package( Boost REQUIRED)

if ( Boost_FOUND )
  message(STATUS "Found Boost library") 
  message(STATUS "\tAdd Include: " ${Boost_INCLUDE_DIRS})
  include_directories(${Boost_INCLUDE_DIRS})
 else()
   message(STATUS "Couldnt find Boost library") 
   return()
endif()

#----------------------------------
# CGAL
find_package(CGAL 4.10 REQUIRED COMPONENTS Core)

if ( CGAL_FOUND )
	#set(GMP_DIR ${CGAL_DIR}/../auxiliary/gmp)
	#set(MPFR_DIR ${CGAL_DIR}/../auxiliary/gmp)
	
	include( ${CGAL_USE_FILE} )
	message(STATUS "CGAL found")
	message(STATUS "\tAdd Include: " ${CGAL_USE_FILE})
else()
	message(SEND_ERROR "Couldnt find CGAL library")
	return()  
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

link_directories(${LIBRARY_OUTPUT_PATH})

FILE(GLOB SRC_FILES src/*.cpp)

add_executable(sfc ${SRC_FILES})

#kövi nem csinál semmit... windows tho
target_link_libraries(sfc ${CGAL_LIBRARIES})

install(
TARGETS sfc 
RUNTIME DESTINATION ${EXECUTABLE_OUTPUT_PATH}
ARCHIVE DESTINATION ${LIBRARY_OUTPUT_PATH}
LIBRARY DESTINATION ${LIBRARY_OUTPUT_PATH}
)