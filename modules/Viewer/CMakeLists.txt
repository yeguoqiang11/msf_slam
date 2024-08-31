cmake_minimum_required(VERSION 3.19)
project(Viewer)

add_library(${PROJECT_NAME} STATIC 
  src/PangoViewer.cc)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${Pangolin_INCLUDE_DIRS})
  
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  utils
  slam)

