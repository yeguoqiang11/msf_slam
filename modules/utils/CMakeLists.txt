cmake_minimum_required(VERSION 3.19)
project(utils)

add_library(${PROJECT_NAME} SHARED 
  src/DMmath.cc
  src/logs.cc
  src/ThreadPool.cc)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
  glog::glog
)

# if (ENABLE_TEST)
#   add_executable(TestUtility test/utility_test.cc)
#   target_include_directories(TestUtility PRIVATE
#     Eigen3::Eigen
#     ${Sophus_INCLUDE_DIRS})
#   target_link_libraries(TestUtility
#     gtest
#     utils)
#   add_test(NAME TestUtility COMMAND ${EXECUTABLE_OUTPUT_PATH}/TestUtility)
# 
#   add_executable(TestRingbuffer test/ringbuffer_test.cc)
#   target_link_libraries(TestRingbuffer
#     gtest
#     utils)
#   add_test(NAME TestRingbuffer COMMAND ${EXECUTABLE_OUTPUT_PATH}/TestRingbuffer)
# 
#   add_executable(TestInterpolateBuffer test/interplation_test.cc)
#   target_include_directories(TestInterpolateBuffer PRIVATE
#     ${Sophus_INCLUDE_DIRS})
#   target_link_libraries(TestInterpolateBuffer
#     gtest
#     utils)
#   add_test(NAME TestInterpolateBuffer COMMAND ${EXECUTABLE_OUTPUT_PATH}/TestInterpolateBuffer)
# endif()
