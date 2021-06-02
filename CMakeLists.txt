cmake_minimum_required(VERSION 3.19)
project(opencv_3d)

set(CMAKE_CXX_STANDARD 14)

#set(OpenCV_DIR ".../opencv/build/x64/vc15/lib")
find_package(OpenCV REQUIRED)

## Information output (not required)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

aux_source_directory(src SRC_FILES)
aux_source_directory(samples SRC_FILES)
set(SRC_FILES "${SRC_FILES};./main.cpp")

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(opencv_3d ${SRC_FILES} ${INCLUDE_DIRECTORIES})

#target_sources(
#        ${SRC_FILES}
#        ${INCLUDE_DIRECTORIES}
#)

target_link_libraries(opencv_3d ${OpenCV_LIBS})

target_include_directories(opencv_3d PUBLIC)

