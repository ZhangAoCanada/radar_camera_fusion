cmake_minimum_required(VERSION 3.0.0)
project(ukf_test VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ukf_test 
src/main.cpp
src/tools.h
src/param.h
src/param.cpp
src/ukf.h
src/ukf.cpp
)

target_link_libraries(ukf_test ${EIGEN3_LIBRARIES})
target_link_libraries(ukf_test ${OpenCV_LIBRARIES})
target_link_libraries(ukf_test ${JSONCPP_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
