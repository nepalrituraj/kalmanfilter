cmake_minimum_required(	VERSION 3.15)

#SET PROJECT NAME
project(kalman VERSION 1.0)

#Add Library
add_library(kalman.cpp kalman.h)

#Add Subdirectory
#add_subdirectory(boost)

add_executibles(main.cpp)


#SPECIFY THE C++ STANDARD
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

target_include_directories(kalman PUBLIC "${PROJECT_BINARY_DIR}}"
"${PROJECT_BINARY_DIR}}/boost")

