
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/Code SOURCE_FILES)

add_executable(PA6_BVHRayTracing ${SOURCE_FILES})

target_compile_features(PA6_BVHRayTracing PUBLIC cxx_std_17)
target_link_libraries(PA6_BVHRayTracing ${OpenCV_LIBS})
