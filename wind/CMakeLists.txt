cmake_minimum_required(VERSION 3.0.0)
project(task)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(Ceres_DIR /usr/local/include/ceres/lib/cmake/Ceres)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

aux_source_directory(./src SRC_FILES)

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

add_executable(task main.cpp ${SRC_FILES})
target_link_libraries(task ${OpenCV_LIBS} ${CERES_LIBRARIES})
