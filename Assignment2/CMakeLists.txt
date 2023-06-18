
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${EIGEN_INCLUDE_DIR})

aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/src SOURCE_FILES)

add_executable(Assignment2 ${SOURCE_FILES})
target_link_libraries(Assignment2 ${OpenCV_LIBS})
