
#config eigen
include_directories(${EIGEN_INCLUDE_DIR})

#config opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
LIST(APPEND LIBS ${OpenCV_LIBS})

#config src code
aux_source_directory(${CMAKE_CURRENT_LIST_DIR} SOURCE_FILES)

add_executable(Assignment1 ${SOURCE_FILES})
target_link_libraries(Assignment1 PRIVATE ${LIBS})
