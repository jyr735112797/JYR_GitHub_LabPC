cmake_minimum_required(VERSION 3.5)

project(interview)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

include_directories(
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

file(GLOB SRCS src/*.cpp)
add_library(${PROJECT_NAME} ${SRCS})

file(GLOB TESTS tests/test_*.cpp)
foreach(test ${TESTS})
    string(REGEX REPLACE "(^.*/|.cpp$)" "" exe ${test})
    add_executable(${exe} ${test})
    target_link_libraries(${exe} ${PROJECT_NAME})
endforeach(test)
