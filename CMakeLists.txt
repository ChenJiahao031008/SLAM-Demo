cmake_minimum_required(VERSION 2.8)
project( TESTFILES )
set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

message(STATUS "[INFO]:opencv is searching....")
find_package( OpenCV REQUIRED)
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${CSPARSE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
)
include_directories( "/usr/include/eigen3" )

add_library(${PROJECT_NAME} SHARED
    src/FeatureMather.cc
    src/EpipolarLine.cc
    src/DepthMap.cc
    src/PoseSolver.cc
    src/Optimization.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${CSPARSE_LIBRARY}
g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(main main.cpp)
target_link_libraries( main ${PROJECT_NAME})
