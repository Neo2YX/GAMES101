find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

set(INCLUDE_DIRS)
set(LIBRARY)
set(COMPILE_OPTIONS)

# ²¢ÐÐ±à³Ì
set (OpenCV_DIR E:/APPinstall/OpenCV4/opencv)
find_package(OpenMP REQUIRED)

list(APPEND INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} )
list(APPEND INCLUDE_DIRS  ${OpenMP_CXX_INCLUDE_DIRS})
list(APPEND LIBRARY ${OpenCV_LIBS} )
list(APPEND LIBRARY  ${OpenMP_CXX_LIBRARIES})
list(APPEND COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})

aux_source_directory( ${CMAKE_CURRENT_LIST_DIR}/Code SOURCE_FILES)

add_executable(PA7_PathTracing ${SOURCE_FILES})

target_include_directories(PA7_PathTracing PRIVATE ${INCLUDE_DIRS})
target_compile_options(PA7_PathTracing PRIVATE ${COMPILE_OPTIONS})
target_link_libraries(PA7_PathTracing PRIVATE ${LIBRARY})
