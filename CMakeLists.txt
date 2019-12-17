cmake_minimum_required( VERSION "3.15.0")
project("kalman")

find_package(Boost 1.71.0 REQUIRED)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  #add_executable(foo foo.cc)
endif()


add_executable("${PROJECT_NAME}" "kalman/main.cpp" "kalman/kalman_2D.cpp" "kalman/kalman_2D.hpp")



install (TARGETS "${PROJECT_NAME}" DESTINATION bin)  
install ( FILES "main.cpp" DESTINATION src ) 

