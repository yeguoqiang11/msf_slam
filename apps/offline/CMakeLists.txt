
cmake_minimum_required(VERSION 3.19)
project(offline)

include_directories(
	${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${GTSAM_LIBRARY_DIRS}
)

add_executable(GTSAMRun src/gtsam_test.cc)
target_link_libraries(GTSAMRun
  ${OpenCV_LIBS}
  Viewer
  dataio
  slam
  gtsam)

add_executable(Machine0 src/Machine0.cc)
target_link_libraries(Machine0
  ${OpenCV_LIBS}
  Viewer
  dataio
  slam
  utils)

add_executable(ImuTest src/Imu_test.cc)
target_link_libraries(ImuTest
  ${OpenCV_LIBS}
  Viewer
  dataio
  slam
  utils
  gtsam)

add_executable(ReadPly src/read_ply.cc)
target_link_libraries(ReadPly
  ${OpenCV_LIBS}
  Viewer
  dataio
  slam
  utils)

  add_executable(WheelTest src/Wheel_test.cc)
  target_link_libraries(WheelTest
    ${OpenCV_LIBS}
    Viewer
    dataio
    slam
    utils)

  add_executable(SunnyMachine src/SunnyMachine.cc)
  target_link_libraries(SunnyMachine
    ${OpenCV_LIBS}
    Viewer
    dataio
    slam
    utils)

  add_executable(MapUpdation src/map_updation_test.cc)
  target_link_libraries(MapUpdation
    ${OpenCV_LIBS}
    Viewer
    dataio
    slam
    utils)

  add_executable(OFilmMachine src/OFilm_test.cc)
  target_link_libraries(OFilmMachine
    ${OpenCV_LIBS}
    Viewer
    dataio
    slam
    utils)

  add_executable(ReadUVC src/ReadUVC.cc)
  target_link_libraries(ReadUVC
    ${OpenCV_LIBS}
    Viewer
    dataio
    slam
    utils)

  add_executable(ImuWheelTest src/ImuWheel_test.cc)
  target_link_libraries(ImuWheelTest
    ${OpenCV_LIBS}
    Viewer
    dataio
    slam
    utils)

  add_executable(GtsamFactor src/gtsam_factor.cc)
  target_link_libraries(GtsamFactor
    ${OpenCV_LIBS}
    Viewer
    dataio
    slam
    gtsam)

  add_executable(ThreadPoolTest src/ThreadPool_test.cc)
  target_link_libraries(ThreadPoolTest
    ${OpenCV_LIBS}
    Viewer
    dataio
    slam
    gtsam)


