cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
project(DWGSIMR LANGUAGES CXX)


set(DWGSIM_RECORD_COMMIT ON CACHE BOOL "record the commit hash")
if(DWGSIM_RECORD_COMMIT)
    execute_process(COMMAND git rev-parse HEAD OUTPUT_FILE ${CMAKE_BINARY_DIR}/commitID.txt WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} TIMEOUT 120 RESULT_VARIABLE RET)
    message(STATUS "git rev-parse returned ${RET}")
    if(NOT RET EQUAL 0)
        message(WARNING "git rev-parse failed")
        set(DWGSIM_RECORDED_COMMIT_HASH "UNKNOWN" CACHE INTERNAL  "Project Public Current Commit Hash")
    else()
        # message(STATUS "current commit id: ")
        # execute_process(COMMAND cat commitID.txt WORKING_DIRECTORY ${CMAKE_BINARY_DIR} TIMEOUT 10)
        file(READ "${CMAKE_BINARY_DIR}/commitID.txt" DWGSIM_RECORDED_COMMIT_HASH)
        string(STRIP ${DWGSIM_RECORDED_COMMIT_HASH} DWGSIM_RECORDED_COMMIT_HASH)
        set(DWGSIM_RECORDED_COMMIT_HASH ${DWGSIM_RECORDED_COMMIT_HASH} CACHE INTERNAL  "Project Public Current Commit Hash")
        message(STATUS "current commit id: ${DWGSIM_RECORDED_COMMIT_HASH}")
    endif()
endif()

find_path(DWGSIM_EXTERNAL_INCLUDE_ARGPARSE argparse REQUIRED PATHS
    externalSrc/argparse/include) # hear-only, no build
find_path(DWGSIM_EXTERNAL_INCLUDE_RAPIDJSON rapidjson REQUIRED PATHS
    externalSrc/rapidjson/include) # hear-only, no build


set(BUILD_SHARED_LIBS FALSE CACHE BOOL "libredwg shared")
set(DWGSIM_LIBREDWG_LIBONLY TRUE CACHE BOOL "libredwg libonly")
add_subdirectory(externalSrc/libredwg)

find_path(DWGSIM_EXTERNAL_INCLUDE_LIBREDWG dwg.h REQUIRED PATHS
    externalSrc/libredwg/include)



set(DWGSIM_EXTERNAL_INCLUDES "")

set(DWGSIM_EXTERNAL_INCLUDES ${DWGSIM_EXTERNAL_INCLUDES}
    ${DWGSIM_EXTERNAL_INCLUDE_LIBREDWG}
    ${DWGSIM_EXTERNAL_INCLUDE_ARGPARSE}
    ${DWGSIM_EXTERNAL_INCLUDE_RAPIDJSON}
    CACHE INTERNAL "dwgsim external")


add_executable(dwgsim ${CMAKE_SOURCE_DIR}/src/dwgsim.cpp)

if(MSVC)
  set(redwg libredwg)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4244 -wd4800 -wd4805 -wd4101 -wd4996 -D_CRT_NONSTDC_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS")
else()
  set(redwg redwg)
endif()
target_link_libraries(dwgsim PUBLIC ${redwg})
target_include_directories(dwgsim PUBLIC  ${DWGSIM_EXTERNAL_INCLUDES})
target_compile_definitions(dwgsim PUBLIC DWGSIM_CURRENT_COMMIT_HASH=${DWGSIM_RECORDED_COMMIT_HASH})