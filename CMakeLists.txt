cmake_minimum_required(VERSION 3.12)
project(assignment1)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(assignment1 main.cpp
        include/CommonHead.h
        include/utils.h src/utils.cpp
        include/LinearLeastSquaresMethod.h src/LinearLeastSquareMethod.cpp
        include/CoordinateTransform.h src/CoordinateTransform.cpp
        include/CalculateDisp.h src/CalculateDisp.cpp
        )

target_link_libraries(${PROJECT_NAME} PRIVATE  quirc opencv_ml opencv_dnn opencv_dpm Eigen3::Eigen libpthread.so)
