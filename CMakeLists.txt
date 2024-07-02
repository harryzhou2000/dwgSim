cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
project(DNDSR LANGUAGES CXX)


find_library(EXTERNAL_LIB_LIBREDWG NAMES libredwg redwg  REQUIRED)
find_path(EXTERNAL_INCLUDE_LIBREDWG dwg.h REQUIRED)
find_path(EXTERNAL_INCLUDE_ARGPARSE argparse REQUIRED PATHS
    externalSrc/argparse/include)


add_executable(dwgsim ${CMAKE_SOURCE_DIR}/src/dwgsim.cpp)

target_link_libraries(dwgsim PUBLIC ${EXTERNAL_LIB_LIBREDWG})
target_include_directories(dwgsim PUBLIC ${EXTERNAL_INCLUDE_LIBREDWG})