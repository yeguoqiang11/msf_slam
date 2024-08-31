cmake_minimum_required(VERSION 3.19)
project(dataio)

add_library(${PROJECT_NAME} STATIC 
  src/DataIO.cc)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR})
  
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  utils)

