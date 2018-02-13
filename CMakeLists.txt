cmake_minimum_required(VERSION 3.2)
project(eigen3-units VERSION 0.0.1 LANGUAGES CXX)
enable_testing()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

message("-- CMAKE_SYSTEM_INFO_FILE: ${CMAKE_SYSTEM_INFO_FILE}")
message("-- CMAKE_SYSTEM_NAME:      ${CMAKE_SYSTEM_NAME}")
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message("-- CMAKE_SYSTEM:           ${CMAKE_SYSTEM}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)

file(GLOB SOURCE_FILES *.cpp)

find_package (Boost REQUIRED)
include_directories( ${Boost_INCLUDE_DIR} )

include_directories("./")
include_directories("./lib/eigen/")

add_executable( eigen3-units ${SOURCE_FILES} )
target_link_libraries( eigen3-units LINK_PUBLIC ${Boost_LIBRARIES} )
add_subdirectory(test)
