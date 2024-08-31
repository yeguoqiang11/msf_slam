cmake_minimum_required(VERSION 3.19)
project(MultiSensorSlam)
# set(CMAKE_CXX_COMPILER clang)
# set(CMAKE_C_COMPILER clang)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
# set(CMAKE_VERBOSE_MAKEFILE 1)
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

option(BUILD_APP "Build Applications" ON)
option(ENABLE_TEST "Enable Unit Test" OFF)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -fPIC -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -fPIC -O3 -march=native")
set(CMAKE_CXX_STANDARD 17)
# set library binary directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/lib)
# set excutable binary directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

include(CheckGit)
CheckGitSetup(${PROJECT_NAME})

find_package(OpenCV REQUIRED)
find_package(Eigen3)
find_package(GTSAM REQUIRED)
find_package(Pangolin REQUIRED)

if (ENABLE_TEST)
  enable_testing()
  add_subdirectory(thirdparty/googletest)
endif ()

# add_subdirectory(thirdparty/rapidjson)
# add_subdirectory(modules/parameter)
add_subdirectory(modules/utils)
add_subdirectory(modules/data_io)
add_subdirectory(modules/slam)
add_subdirectory(modules/Viewer)
add_subdirectory(thirdparty/glog)
if (BUILD_APP)
#   online版本
#   add_subdirectory(apps/ava_node)
# 
#  离线仿真
  add_subdirectory(apps/offline)
endif()


