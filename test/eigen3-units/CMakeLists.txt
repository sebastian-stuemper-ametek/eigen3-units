FILE(GLOB SRCS *.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall -fprofile-arcs -ftest-coverage")
# set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)

ADD_EXECUTABLE(test-eigen3-units ${SRCS})

TARGET_LINK_LIBRARIES(test-eigen3-units
	libgtest
	libgmock
)

add_test(NAME test-eigen3-units	COMMAND test-eigen3-units)
