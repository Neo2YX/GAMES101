
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${EIGEN_INCLUDE_DIR})

aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/Code SOURCE_FILES)

add_executable(Assignment3 ${SOURCE_FILES})
target_link_libraries(Assignment3 ${OpenCV_LIBS})
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)