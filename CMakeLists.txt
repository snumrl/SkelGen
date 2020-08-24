cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)
project(editor)

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()


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

#file(GLOB srcs
#		"functions/*.h" "functions/*.cpp"
#		"generator/*.h" "generator/*.cpp"   
#		"generator/motionAdjust/*.h" "generator/motionAdjust/*.cpp"
#		"model/*.h" "model/*.cpp"
#		"render/*.h" "render/*.cpp"
#		"world/*.h" "world/*.cpp"
#		"simpleMotion/*.h" "simpleMotion/*.cpp"
#		"visualROM/*.h" "visualROM/*.cpp"
#		"exams/*.h" "exams/*.cpp"
#		"romEditor/*.h" "romEditor/*.cpp")
#find_package(DART REQUIRED COMPONENTS gui optimizer-ipopt CONFIG)
#add_compile_options(-DHAVE_CSTDDEF)
#include_directories(${Boost_INCLUDE_DIRS})
#include_directories(${DART_INCLUDE_DIRS})

#add_executable(skelgen ${srcs})
#target_link_libraries(skelgen ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${DART_LIBRARIES} ${Boost_LIBRARIES} /usr/lib/x86_64-linux-gnu/libtinyxml.so stdc++fs)

file(GLOB srcs
		"functions/*.h" "functions/*.cpp"
		"generator/*.h" "generator/*.cpp"
		"generator/motionAdjust/*.h" "generator/motionAdjust/*.cpp"
		"model/*.h" "model/*.cpp"
		"render/*.h" "render/*.cpp"
		"world/*.h" "world/*.cpp"
		"simpleMotion/*.h" "simpleMotion/*.cpp"
		"visualROM/*.h" "visualROM/*.cpp"
		"editor/*.h" "editor/*.cpp"
		"romEditor/*.h" "romEditor/*.cpp"
		"APTOpt/*.h" "APTOpt/*.cpp")

find_package(DART REQUIRED COMPONENTS gui optimizer-ipopt CONFIG)
add_compile_options(-DHAVE_CSTDDEF)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${DART_INCLUDE_DIRS})

add_executable(editor ${srcs})
target_link_libraries(editor ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${DART_LIBRARIES} ${Boost_LIBRARIES} /usr/lib/x86_64-linux-gnu/libtinyxml.so stdc++fs)
