cmake_minimum_required(VERSION 2.8.6)

SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#	FORCE
	)


add_compile_options(-DHAVE_CSTDDEF)

find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)

add_compile_options(-DHAVE_CSTDDEF)

include_directories(${TinyXML_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})


add_executable(skel_parser "skel_main.cpp")
target_link_libraries(skel_parser ${TinyXML_LIBRARIES})

add_executable(muscle_parser "muscle_main.cpp")
target_link_libraries(muscle_parser ${TinyXML_LIBRARIES})

