find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${EIGEN_INCLUDE_DIR})

#add_executable(testOpenCV testOpenCV.cpp)
#target_link_libraries(testOpenCV PRIVATE ${OpenCV_LIBS})

add_executable(testEigen testEigen.cpp)