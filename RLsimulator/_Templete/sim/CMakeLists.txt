cmake_minimum_required(VERSION 2.8.6)
project(sim)

SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#	FORCE
	)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

find_package(DART REQUIRED COMPONENTS collision-bullet optimizer-ipopt)
find_package(TinyXML REQUIRED)
find_package(Boost 1.66.0 COMPONENTS filesystem)

include_directories(${DART_INCLUDE_DIRS})
include_directories(${TinyXML_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

file(GLOB srcs "*.h" "*.cpp")

add_library(sim ${srcs})
target_link_libraries(sim ${DART_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES})
