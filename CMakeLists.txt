cmake_minimum_required(VERSION 3.0)
project(vri-slam-fast)

find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} )

find_package(OpenCV REQUIRED)

add_executable(vri-slam 
    ./src/main.cpp 
    ./src/movementcontroller.cpp
    ./src/kalmanfilter.cpp 
    ./src/kalmanfilter_update.cpp
    ./src/kalmanfilter_propagate.cpp
    ./src/featuredetector.cpp
    ./src/houghtransform.cpp
    )

target_link_libraries(vri-slam lt_api ${OpenCV_LIBS})

install(TARGETS vri-slam DESTINATION bin)