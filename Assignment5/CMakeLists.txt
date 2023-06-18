find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/Code SOURCE_FILE)

add_executable(RayTracing_Assignment5 ${SOURCE_FILE})
#target_compile_options(RayTracing_Assignment5 PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type -fsanitize=undefined)
target_compile_features(RayTracing_Assignment5 PUBLIC cxx_std_17)
target_link_libraries(RayTracing_Assignment5 ${OpenCV_LIBS})
#target_link_libraries(RayTracing_Assignment5 PUBLIC -fsanitize=undefined)
