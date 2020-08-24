cmake_minimum_required(VERSION 2.8)
project(converter)
add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)

SET(CMAKE_BUILD_TYPE Release CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
		# FORCE
		)
add_definitions(-DMSS_ROOT_DIR="${CMAKE_HOME_DIRECTORY}")

add_definitions(-DOPT_MODE="LINE_SEARCH")
#add_definitions(-DOPT_MODE="GRADIENT_DESCENT")

#add_definitions(-DXML_MODE="RHS")
add_definitions(-DXML_MODE="LSW")

include(FindOpenMP)

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Boost 1.45.0 COMPONENTS filesystem)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR})

file(GLOB srcs main.cpp)
find_package(DART REQUIRED COMPONENTS gui optimizer-ipopt CONFIG)
add_compile_options(-DHAVE_CSTDDEF)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${DART_INCLUDE_DIRS})

add_executable(converter ${srcs})
target_link_libraries(converter ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${DART_LIBRARIES} ${Boost_LIBRARIES} /usr/lib/x86_64-linux-gnu/libtinyxml.so stdc++fs)
