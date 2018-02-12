file(GLOB SRCS *.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall")
# set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)

find_package (Boost REQUIRED)
include_directories( ${Boost_INCLUDE_DIR} )

add_executable(test-eigen3-units ${SRCS})

target_link_libraries(test-eigen3-units
	libgtest
	libgmock
	${Boost_LIBRARIES}
)

add_test(NAME test-eigen3-units	COMMAND test-eigen3-units)
