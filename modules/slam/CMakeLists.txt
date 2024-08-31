cmake_minimum_required(VERSION 3.19)
project(slam)

include_directories(
	${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${GTSAM_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/SlamSystem.cc
  src/DepthMapping.cc
  src/VoxelMap.cc
  src/Map.cc
  src/KLOdometry.cc
  src/Imu.cc
  src/PCLoopClosure.cc
  src/ContourMap.cc
  src/Attitude.cc
  src/Sensors.cc
  src/WheelModel.cc
  src/ImuWheel.cc
  src/PoseGraph.cc
  src/Shower.cc
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  utils
  gtsam)
# 
# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# set_target_properties(${PROJECT_NAME} PROPERTIES
#   POSITION_INDEPENDENT_CODE True)
