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
    externalSrc/argparse/include NO_DEFAULT_PATH) # hear-only, no build
find_path(DWGSIM_EXTERNAL_INCLUDE_RAPIDJSON rapidjson REQUIRED PATHS
    externalSrc/rapidjson/include NO_DEFAULT_PATH) # hear-only, no build
find_path(DWGSIM_EXTERNAL_INCLUDE_EIGEN Eigen REQUIRED  PATHS
    externalSrc/eigen NO_DEFAULT_PATH) # hear-only, no build

set(BUILD_SHARED_LIBS OFF CACHE BOOL "libredwg shared")
set(LIBREDWG_LIBONLY ON CACHE BOOL "libredwg libonly")
set(DISABLE_WERROR ON CACHE BOOL "libredwg no werror")
add_subdirectory(externalSrc/libredwg)

find_path(DWGSIM_EXTERNAL_INCLUDE_LIBREDWG dwg.h REQUIRED PATHS
    externalSrc/libredwg/include)



set(DWGSIM_EXTERNAL_INCLUDES src)

set(DWGSIM_EXTERNAL_INCLUDES ${DWGSIM_EXTERNAL_INCLUDES}
    ${DWGSIM_EXTERNAL_INCLUDE_LIBREDWG}
    ${DWGSIM_EXTERNAL_INCLUDE_ARGPARSE}
    ${DWGSIM_EXTERNAL_INCLUDE_RAPIDJSON}
    ${DWGSIM_EXTERNAL_INCLUDE_EIGEN}
    CACHE INTERNAL "dwgsim external")


set(DWGSIM_CPPS ""
src/dwgsimReader.cpp
)

add_executable(dwgsim src/dwgsim.cpp ${DWGSIM_CPPS})

add_executable(testSplineConversion test/testSplineConversion.cpp ${DWGSIM_CPPS})

set(exeTargets dwgsim
testSplineConversion
)

if(MSVC)
  set(redwg libredwg)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4819 -wd4244 -wd4800 -wd4805 -wd4101 -wd4996 -D_CRT_NONSTDC_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS")
else()
  set(redwg redwg)
endif()

foreach(t IN LISTS exeTargets)
    message(STATUS ${t})
    target_link_libraries(${t} PUBLIC ${redwg})
    target_include_directories(${t} PUBLIC  ${DWGSIM_EXTERNAL_INCLUDES})
    target_compile_definitions(${t} PUBLIC DWGSIM_CURRENT_COMMIT_HASH=${DWGSIM_RECORDED_COMMIT_HASH})
endforeach()

