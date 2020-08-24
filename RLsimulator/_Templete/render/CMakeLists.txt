cmake_minimum_required(VERSION 2.8.6)
project(render)

add_compile_options(-fPIC)
add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)
SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#	FORCE
	)

link_directories(../sim/)
include_directories(../sim/)
find_package(DART REQUIRED COMPONENTS gui collision-bullet optimizer-ipopt CONFIG)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

find_package(PythonLibs REQUIRED)
find_package(Boost 1.66.0 COMPONENTS filesystem python3 numpy3)

add_compile_options(-DHAVE_CSTDDEF)
include_directories(${DART_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
file(GLOB srcs "*.h" "*.cpp")

add_executable(render ${srcs})
target_link_libraries(render ${DART_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} GL GLU glut sim /usr/lib/x86_64-linux-gnu/libtinyxml.a)
