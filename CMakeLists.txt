cmake_minimum_required(VERSION 3.8)

project(GAMES101_On_Windows VERSION 1.0
                            DESCRIPTION "build games101 assignments on Windows"
                            LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/lib/shared)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

# set eigen
set(EIGEN_INCLUDE_DIR ${ROOT_DIR}/Dependencies/eigen3)


#add_subdirectory(Test)
#add_subdirectory(Assignment0)
#add_subdirectory(Assignment1)
#add_subdirectory(Assignment2)
#add_subdirectory(Assignment3)
#add_subdirectory(Assignment4)
#add_subdirectory(Assignment5)
add_subdirectory(Assignment6)
#add_subdirectory(Assignment7)