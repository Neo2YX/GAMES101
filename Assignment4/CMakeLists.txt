
find_package(OpenCV REQUIRED)

add_executable(BezierCurve_Assignment4 main.cpp)

target_link_libraries(BezierCurve_Assignment4 ${OpenCV_LIBRARIES})
